"""
@file run_agent.py
@brief CLI script — instantiate and run a LOTUSim agent locally.

The agent node runs on THIS machine and communicates with the Gazebo simulation
over the ROS 2 network (same ROS_DOMAIN_ID).  Press Ctrl+C to despawn and exit.

Usage::

    ros2 run lotusim_client run_agent --world <world_name> --config path/to/agent.json
    ros2 run lotusim_client run_agent --world energy --json '{"Bluerov2HeavyBase": {"pose": [0,0,-10,0,0,0]}}'

JSON format (same as scenario configs)::

    {
      "MyAgentClass": {
        "pose": [x, y, z, roll, pitch, yaw],   # OR lat/lon
        "ais": {"patrol_file": "WP-WT1.json"},  # optional, loads trajectory
        "loop": true
      }
    }

Multi-patrol (one trajectory per agent)::

    {
      "MyAgentClass": {
        "nb_agents": 3,
        "ais": {"patrol_file": ["WP1.json", "WP2.json", "WP3.json"]},
      }
    }
"""

import argparse
import json
import math
import os
import signal
import sys
import time

import rclpy
from rclpy.executors import MultiThreadedExecutor


# ---------------------------------------------------------------------------
# Geo -> ENU projection
#
# The MAS entity manager IGNORES the altitude field of a GeoPoint when spawning,
# so a geographic spawn must be converted to a 6-element ENU pose [x, y, z, 0,0,0]
# using the world's geographic origin (<spherical_coordinates> in the world file).
# This mirrors simulation_run.utils._latlon_to_enu_xy on the host side.
# ---------------------------------------------------------------------------

_EARTH_RADIUS_M = 6_371_000.0


def _latlon_to_enu_xy(lat: float, lon: float, lat0: float, lon0: float):
    lat0_rad = math.radians(lat0)
    x = math.radians(lon - lon0) * math.cos(lat0_rad) * _EARTH_RADIUS_M
    y = math.radians(lat - lat0) * _EARTH_RADIUS_M
    return x, y


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _load_json_file(path: str) -> dict:
    with open(path) as f:
        return json.load(f)


from lotusim_sdk.spawn_utils import extract_spawn_from_missions as _extract_spawn_from_missions


def _find_agent_class(class_name: str):
    """Discover an agent class from installed packages via the 'lotusim.agents' entry_point group."""
    try:
        from importlib.metadata import entry_points
        eps = entry_points(group="lotusim.agents")
        for ep in eps:
            try:
                discovered = ep.load()
                if discovered.__name__ == class_name:
                    return discovered
            except Exception:
                continue
    except Exception as e:
        print(f"Entry point discovery error: {e}", file=sys.stderr)
    return None


def _resolve_pose(agent_info: dict, origin, agent_index: int = 0):
    """
    Return the best available spawn pose for agent number *agent_index*.

    Priority: explicit ``spawn`` block > ``poses`` list > ``pose`` > lat/lon fallback > origin.
    z defaults to 0.0 when not specified.
    """
    # --- Explicit ENU spawn block ---
    spawn = agent_info.get("spawn")
    if spawn is not None:
        return [
            float(spawn.get("x", 0.0)),
            float(spawn.get("y", 0.0)),
            float(spawn.get("z", 0.0)),
            float(spawn.get("roll", 0.0)),
            float(spawn.get("pitch", 0.0)),
            float(spawn.get("yaw", 0.0)),
        ]

    poses = agent_info.get("poses")
    if poses:
        idx = agent_index if agent_index < len(poses) else 0
        return poses[idx]

    pose = agent_info.get("pose")
    if pose:
        return pose

    if "lat" in agent_info:
        if origin is not None:
            lat0, lon0 = origin
            x, y = _latlon_to_enu_xy(
                float(agent_info["lat"]), float(agent_info["lon"]), lat0, lon0
            )
            return [x, y, 0.0, 0.0, 0.0, 0.0]
        print(
            'WARNING: no world origin provided (--origin or "origin" in config); '
            "spawning via GeoPoint.",
            file=sys.stderr,
        )
        return [float(agent_info["lat"]), float(agent_info["lon"]), 0.0]

    return [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="Run a LOTUSim agent locally — spawns in Gazebo via ROS 2 network."
    )
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--config", help="Path to agent JSON config file")
    group.add_argument("--json", dest="json_str", help="Inline JSON agent config")
    parser.add_argument(
        "--world", required=True,
        help="Simulation world name, e.g. 'energy' (must match the running scenario)"
    )
    parser.add_argument(
        "--origin", nargs=2, type=float, metavar=("LAT", "LON"), default=None,
        help="World geographic origin (latitude longitude) used to convert a "
             "lat/lon spawn into an ENU pose. Read it from "
             "the world file's <spherical_coordinates> block. Overrides an 'origin' "
             "key in the config. Without it, the agent spawns via GeoPoint.",
    )
    args = parser.parse_args()

    # Load config
    config_dir = None
    if args.config:
        config_dir = os.path.dirname(os.path.abspath(args.config))
        try:
            data = _load_json_file(args.config)
        except (FileNotFoundError, json.JSONDecodeError) as e:
            print(f"Config error: {e}", file=sys.stderr)
            sys.exit(1)
    else:
        try:
            data = json.loads(args.json_str)
        except json.JSONDecodeError as e:
            print(f"Invalid JSON: {e}", file=sys.stderr)
            sys.exit(1)

    if not data or not isinstance(data, dict):
        print("Config must be a non-empty JSON object", file=sys.stderr)
        sys.exit(1)

    # Optional top-level world origin shared by all agents (CLI flag takes priority).
    global_origin = data.pop("origin", None)

    # Process all agents in the config
    agents = []

    rclpy.init()
    executor = MultiThreadedExecutor()

    # Normalize both config shapes into (name_base, class_name, agent_info) specs:
    #   - mission (list) form:  {"agents": [{"id", "class", "missions", ...}]}
    #   - legacy inline form:   {"AgentClass": {"ais": ..., "pose": ...}, ...}
    raw_agents = data.get("agents")
    if isinstance(raw_agents, list):
        agent_specs = [
            (
                ai.get("id") or (ai.get("class") or "agent").lower(),
                ai.get("class") or ai.get("type") or ai.get("id"),
                ai,
            )
            for ai in raw_agents
            if isinstance(ai, dict)
        ]
    else:
        agent_specs = [
            (agent_key.lower(), agent_info.get("type", agent_key), agent_info)
            for agent_key, agent_info in data.items()
            if isinstance(agent_info, dict)
        ]

    for name_base, agent_type, agent_info in agent_specs:
        try:
            _extract_spawn_from_missions(agent_info, config_dir)
        except Exception as e:
            print(f"Error extracting spawn for {name_base}: {e}", file=sys.stderr)
            continue

        agent_class = _find_agent_class(agent_type)
        if agent_class is None:
            print(
                f"Agent class '{agent_type}' not found for agent '{name_base}'.\n"
                f"Make sure your package is installed and declares it under the "
                f"'lotusim.agents' entry_point group in setup.py.",
                file=sys.stderr,
            )
            continue

        sdf_file = agent_info.get("sdf_file", "")
        xdyn_enabled = bool(agent_info.get("xdyn", False))
        missions = agent_info.get("missions")
        tick_rate_hz = float(agent_info.get("tick_rate_hz", 1.0))
        nb_agents = int(agent_info.get("nb_agents", 1))

        for i in range(nb_agents):
            agent = agent_class(sdf_file, args.world, xdyn_enabled)

            # Always derive the agent name from the id/class + index, matching the
            # host-side naming convention (e.g. "tester" → "tester0", "tester1").
            # This ensures a stable, predictable name regardless of class-internal counters,
            # and avoids collisions when multiple processes spawn agents of the same class.
            agent.agent_name = f"{name_base}{i}"

            # Route the config's "sdf_file" to the attribute the entity_manager
            # actually reads (cmd.sdf_file) to pick which SDF inside the model
            # folder to spawn (e.g. "model-battery.sdf").  The constructor's first
            # positional is sdf_string, NOT sdf_file, so without this the JSON key
            # never reaches the spawn command and the default model.sdf is used.
            if sdf_file:
                agent.sdf_file = sdf_file

            # Expose the world geographic origin so a WaypointFollowerTask can
            # project its lat/lon waypoints into the same world ENU frame Gazebo
            # uses for the poses on /<world>/poses. Set before set_missions so it
            # is in place by the time the mission ticks (and the task reads it).
            if config_dir is not None:
                agent._config_dir = config_dir

            early_origin = args.origin or agent_info.get("origin") or global_origin
            if early_origin is not None:
                agent._world_origin = (float(early_origin[0]), float(early_origin[1]))

            # Install behaviour-tree missions (mission system) — the tick timer
            # drives them once the executor starts spinning.
            if missions and hasattr(agent, "set_missions"):
                agent.set_missions(missions, tick_rate_hz)

            executor.add_node(agent)
            agents.append((agent, agent_info, i))

    if not agents:
        print("No valid agents were spawned. Exiting.", file=sys.stderr)
        rclpy.shutdown()
        sys.exit(1)

    # Spin briefly so the poses subscription can receive existing entity positions.
    # This replaces the per-agent sleep: one shared wait is enough for DDS
    # discovery AND for detecting agents that are already present in Gazebo.
    print("Waiting for network discovery to stabilize...")
    deadline = time.time() + 2.0
    while time.time() < deadline:
        executor.spin_once(timeout_sec=0.1)

    # Dispatch every agent's CREATE_CMD without blocking on its confirmation: the
    # host (entity_manager) drains all pending CREATE_CMDs once per Gazebo
    # PreUpdate step, so spawn confirmation latency tracks the simulation's
    # real-time factor, not the network. RTF drops as the world fills up, so a
    # short fixed per-agent wait here would "time out" spawns the host is still
    # going to confirm seconds later — a false failure, since the host never
    # drops a queued CREATE_CMD (it has no expiry and cancellation is rejected
    # server-side). All agents share one MASCmd ActionClient per process (see
    # entity.py's _get_shared_mas_client), which tracks every outstanding goal by
    # UUID, so firing goals back-to-back is safe — no cross-routing risk.
    pending_agents = []
    for agent, agent_info, agent_idx in agents:
        # Origin precedence: CLI flag > per-agent "origin" > top-level "origin".
        origin = args.origin or agent_info.get("origin") or global_origin
        pose = _resolve_pose(agent_info, origin, agent_index=agent_idx)

        # Reuse is OPT-IN ("reuse_existing": true in the agent config). Auto-reuse
        # based on seeing a pose for our name is unsafe across machines: a pose for
        # "mybluerov0" on /poses may belong to ANOTHER machine's same-named entity,
        # not one we previously spawned. If we reuse it we never send CREATE_CMD,
        # so the host never deconflicts our name and both machines end up driving
        # the same /{world}/mybluerov0/... topics. Default is therefore to always
        # spawn our own entity and adopt the host-assigned (possibly deconflicted)
        # name. Set "reuse_existing": true only for the single-machine restart case
        # where you want to reattach to a still-living entity instead of respawning.
        reuse_existing = bool(agent_info.get("reuse_existing", False))
        if reuse_existing and agent.current_pose is not None:
            # The agent is already present in Gazebo (its pose appeared on the
            # /poses topic during the discovery window).  Skip the CREATE_CMD —
            # a WaypointFollowerTask keeps publishing velocity set-points on
            # /<world>/vessel_cmd_array regardless. No CREATE_CMD means no host
            # confirmation, so mark the spawn confirmed here (the entity is
            # known-present) to let the gated missions start.
            print(f"[REUSING]  {agent.agent_name} already exists in Gazebo.")
            agent._spawn_confirmed = True
            continue

        print(f"Spawning {agent.agent_name} in world '{args.world}'...")
        # send_single_mas_cmd attaches the spawn-result handler in the SDK: it
        # adopts the host-assigned (possibly deconflicted) name into agent_name and
        # flips _spawn_confirmed, so the gated missions only start once THIS agent's
        # own entity exists under its final name. We just report accept/reject.
        requested = agent.agent_name
        future = agent.send_single_mas_cmd(pose)
        if future is None:
            continue

        def goal_response_callback(fut, requested=requested):
            goal_handle = fut.result()
            if goal_handle.accepted:
                print(f"[ACCEPTED] The simulation host ACCEPTED the spawn request for {requested}.")
            else:
                print(f"[REJECTED] The simulation host REJECTED the spawn request for {requested}!", file=sys.stderr)

        future.add_done_callback(goal_response_callback)
        pending_agents.append((agent, requested))

        # A short, non-blocking spin so ACCEPTED/REJECTED prints and any already-
        # available results surface as we go, without waiting for confirmation.
        executor.spin_once(timeout_sec=0.1)

    # Wait for every dispatched spawn to confirm (or definitively fail). There is
    # no fixed per-agent timeout: as long as confirmations keep arriving anywhere
    # in the batch, the host is making progress and we keep waiting, so a slow
    # host never produces a false "failed" log. Only once nothing has confirmed
    # for STALL_TIMEOUT seconds straight do we conclude those agents are
    # genuinely stuck and report it.
    STALL_TIMEOUT = 30.0
    last_progress = time.time()
    while pending_agents:
        before = len(pending_agents)
        executor.spin_once(timeout_sec=0.1)
        pending_agents = [(a, n) for a, n in pending_agents if not a._spawn_confirmed]
        if len(pending_agents) < before:
            last_progress = time.time()
        elif time.time() - last_progress > STALL_TIMEOUT:
            break

    if pending_agents:
        names = ", ".join(n for _, n in pending_agents)
        print(
            f"[WARN] {len(pending_agents)} agent(s) never confirmed and the host "
            f"made no progress for {STALL_TIMEOUT:.0f}s: {names}. Continuing "
            "without them.",
            file=sys.stderr,
        )

    print(f"\n{len(agents)} agent(s) initialized. Press Ctrl+C to despawn and exit.")

    # Override rclpy's default SIGINT/SIGTERM handlers so that Ctrl+C does NOT
    # call rclpy.shutdown() immediately.  If rclpy shuts down before our finally
    # block, all subsequent ROS 2 calls (wait_for_server, send_goal_async, …)
    # silently fail and the delete command is never transmitted.
    _stop = False

    def _request_stop(signum, frame):
        nonlocal _stop
        _stop = True

    signal.signal(signal.SIGINT, _request_stop)
    signal.signal(signal.SIGTERM, _request_stop)

    try:
        while not _stop:
            executor.spin_once(timeout_sec=0.2)
    except Exception:
        pass
    finally:
        print("\nStopping agent tasks...")
        for agent, _, _ in agents:
            # Stop behaviour-tree missions: cancel the tick timer and halt the
            # trees so any RUNNING leaf gets its on_exit(FAILURE) cleanup.
            timer = getattr(agent, "_mission_timer", None)
            if timer is not None:
                try:
                    timer.cancel()
                except Exception:
                    pass
            for root in getattr(agent, "_missions", []):
                try:
                    root.halt()
                except Exception:
                    pass

        # Despawn each agent's entity from the simulation host.  Without this the
        # spawned model lives on in Gazebo and its host-side plugins keep
        # publishing/subscribing forever (battery_sensor → battery/state,
        # ais_sensor → ais, light_actuator → light/cmd), so those topics survive
        # the agent process exit.  DELETE_CMD removes the entity → topics drop.
        print("Despawning agents...")
        delete_futures = []
        for agent, _, _ in agents:
            try:
                fut = agent.send_single_delete_cmd()
                if fut is not None:
                    delete_futures.append(fut)
            except Exception:
                pass

        # Spin until the delete goals are accepted (or a deadline), so both the
        # on_detach() service calls and the DELETE_CMDs are actually transmitted
        # over DDS before we shut down.
        deadline = time.time() + 5.0
        while time.time() < deadline:
            if delete_futures and all(f.done() for f in delete_futures):
                break
            try:
                executor.spin_once(timeout_sec=0.1)
            except Exception:
                break

        try:
            executor.shutdown()
            rclpy.shutdown()
        except Exception:
            pass
        print("Done.")

if __name__ == "__main__":
    main()
