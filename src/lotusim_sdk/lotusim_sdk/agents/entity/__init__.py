import logging
import math
import threading
import time
import uuid

import lotusim_msgs.action
from geometry_msgs.msg import Pose
from geographic_msgs.msg import GeoPoint
from lotusim_msgs.msg import MASCmd, VesselPositionArray
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rosidl_runtime_py.utilities import get_message
from std_msgs.msg import String

from lotusim_sdk.agents.agent import Agent


# Module-level registry of MASCmd action clients, one per world. Every agent in a
# process shares the SAME ActionClient for "/{world}/mas_cmd" instead of each
# creating its own.
#
# Multiple ActionClient instances bound to the same action name inside one process
# make rclpy cross-route goal/result responses between them ("Ignoring unexpected
# goal/result response. There may be more than one action server ..."): a result
# meant for one agent resolves another agent's future, so agents adopt the wrong
# assigned name (duplicate Gazebo entities) or never confirm (missed spawns). The
# failure grows with the number of agents, which is why spawning ~20+ at once is
# unreliable even though the launcher serializes the sends. A single ActionClient
# tracks all its outstanding goals by UUID internally and routes each result to
# the correct goal future, removing the race.
#
# Agents stay autonomous and decentralized: each still builds and sends its OWN
# CREATE_CMD / DELETE_CMD and confirms ITSELF (the spawn-result handler captures
# the calling agent). The shared client is purely a per-process rmw endpoint — it
# is NOT a central spawner. Separate processes / remote machines each keep their
# own client, so multi-machine spawns are unaffected. The client is bound to the
# FIRST agent's node for that world (the "owner"); since every agent runs in the
# same executor, the owner node is spun and the shared client's callbacks are
# serviced no matter which agent sent the goal.
_shared_mas_clients: dict[str, ActionClient] = {}
_shared_mas_clients_lock = threading.Lock()

# Dedicated callback group for shared per-process endpoints (MASCmd client and
# the pose subscription below), so their callbacks are NOT serialized behind
# any one owner node's default MutuallyExclusiveCallbackGroup (which also
# carries that node's own mission/discovery timers). Reentrant lets the
# MultiThreadedExecutor service these callbacks without queuing behind
# unrelated per-agent work.
_shared_callback_group = ReentrantCallbackGroup()


def _get_shared_mas_client(node, world_name: str) -> ActionClient:
    """Return the process-wide MASCmd ActionClient for ``world_name``, creating it
    (bound to ``node``) on first use."""
    with _shared_mas_clients_lock:
        client = _shared_mas_clients.get(world_name)
        if client is None:
            client = ActionClient(
                node, lotusim_msgs.action.MASCmd, f"/{world_name}/mas_cmd",
                callback_group=_shared_callback_group,
            )
            _shared_mas_clients[world_name] = client
        return client


# Process-wide MASCmdArray ("/{world}/mas_cmd_array") client, one per world —
# mirrors the single-cmd client above. The host entity_manager accepts one goal
# carrying N CREATE_CMDs and returns all assigned names in one Result, so batching
# a whole spawn wave into a single goal yields one acceptance and one result for the
# wave instead of N. Callers fall back to per-agent single sends when this server is
# absent.
_shared_mas_array_clients: dict[str, ActionClient] = {}


def _get_shared_mas_array_client(node, world_name: str) -> ActionClient:
    """Return the process-wide MASCmdArray ActionClient for ``world_name``, creating
    it (bound to ``node``) on first use."""
    with _shared_mas_clients_lock:
        client = _shared_mas_array_clients.get(world_name)
        if client is None:
            client = ActionClient(
                node, lotusim_msgs.action.MASCmdArray, f"/{world_name}/mas_cmd_array",
                callback_group=_shared_callback_group,
            )
            _shared_mas_array_clients[world_name] = client
        return client


def send_batch_mas_cmd(entries, server_timeout_sec: float = 5.0) -> bool:
    """Spawn a whole wave of agents with ONE MASCmdArray goal per world.

    ``entries`` is a list of ``(entity, value)`` pairs, where ``value`` is an ENU
    pose ``[x, y, z, roll, pitch, yaw]`` or a geographic ``[lat, lon]`` /
    ``[lat, lon, alt]``. All the CREATE_CMDs for a world are packed into a single
    goal; the host returns every assigned name in one Result (order preserved), so
    each entity adopts its host-assigned name and flips ``_spawn_confirmed`` exactly
    as the single-cmd path does — but with ONE acceptance and ONE result instead of
    N. This removes the per-send acceptance-drain entirely.

    Returns ``True`` if the batch goal(s) were dispatched. Returns ``False`` without
    sending anything if any world's ``mas_cmd_array`` server is unavailable, so the
    caller can fall back to per-agent single sends.
    """
    entries = [(e, v) for e, v in entries if e is not None]
    if not entries:
        return True

    # Group by world; each world has its own array server and client.
    by_world: dict[str, list] = {}
    for entity, value in entries:
        by_world.setdefault(entity.world_name, []).append((entity, value))

    # Verify EVERY world's server is up before sending any goal, so a missing
    # server never leaves half the wave batched and the other half to the caller's
    # fallback (which would re-CREATE the already-sent half → duplicates).
    clients: dict[str, ActionClient] = {}
    for world, group in by_world.items():
        owner = group[0][0]
        client = _get_shared_mas_array_client(owner, world)
        if not client.wait_for_server(timeout_sec=server_timeout_sec):
            owner.get_logger().warning(
                f"MASCmdArray server for world '{world}' unavailable; "
                "falling back to per-agent spawn."
            )
            return False
        clients[world] = client

    for world, group in by_world.items():
        group_entities = [e for e, _ in group]
        goal_msg = lotusim_msgs.action.MASCmdArray.Goal()
        goal_msg.cmd = [e._build_create_cmd(v) for e, v in group]

        def _on_goal(gf, entities=group_entities):
            try:
                goal_handle = gf.result()
            except Exception:
                entities[0].get_logger().error(
                    "batch spawn: goal future.result() raised:", exc_info=True
                )
                return
            if goal_handle is None or not goal_handle.accepted:
                for e in entities:
                    e.get_logger().error(f"{e.agent_name}: batch spawn REJECTED by host.")
                return
            for e in entities:
                e._spawn_goal_accepted = True

            def _on_result(rf, entities=entities):
                try:
                    res = rf.result().result
                except Exception:
                    entities[0].get_logger().error(
                        "batch spawn: result future.result() raised:", exc_info=True
                    )
                    return
                names = list(res.name)
                for i, e in enumerate(entities):
                    assigned = names[i] if i < len(names) else ""
                    if not assigned or assigned == "error_cmd":
                        e.get_logger().error(f"{e.agent_name}: host failed to spawn entity.")
                        continue
                    e.confirm_spawn(assigned)

            goal_handle.get_result_async().add_done_callback(_on_result)

        clients[world].send_goal_async(goal_msg).add_done_callback(_on_goal)

    return True


# Module-level registry of the "/{world}/poses" subscription, one per world,
# shared by every agent in the process — mirroring the MASCmd client above.
#
# Previously EVERY Entity created its OWN subscription to this topic and
# linearly scanned msg.vessels for its own name on every message. The message
# carries ALL vessels in the world, so with N agents in one process that is an
# O(N) scan repeated by N separate subscriptions per message = O(N^2) Python
# work under the GIL (on top of DDS having to fan the same payload out to N
# subscriber endpoints). At a few hundred agents this saturates the
# interpreter and starves every OTHER callback in the process — including the
# MASCmd spawn-result callbacks — which is what caused most spawns to never
# confirm at ~450 agents even though the host had created and confirmed all of
# them. A single shared subscription parses the array once per message into a
# name -> pose dict; each entity then does an O(1) lookup by its own name via
# the current_pose/last_pose_update properties below.
_shared_pose_tables: dict[str, dict[str, Pose]] = {}
_shared_pose_stamps: dict[str, float] = {}
# Latest /<world>/poses header stamp, in SIMULATION seconds (Gazebo sim time,
# set host-side in entity_manager::publishPose). Distinct from _shared_pose_stamps
# (wall-clock time.time()): control loops pace their dt off this sim clock so
# their ramps are independent of the real-time factor.
_shared_pose_sim_time: dict[str, float] = {}
_shared_pose_subscriptions: dict[str, object] = {}
_shared_pose_registry_lock = threading.Lock()

# Callbacks fired (in registration order) after every /<world>/poses message is
# parsed into the shared tables above. A control loop registers here to run once
# per published pose (one per Gazebo physics step) instead of off a wall-clock
# timer, giving an update rate independent of the real-time factor. See
# WaypointFollowerTask's "pose" guidance clock.
_shared_pose_listeners: dict[str, list] = {}


def register_pose_listener(world_name: str, cb) -> None:
    """Register ``cb`` (a no-arg callable) to run after each /<world>/poses
    message. Safe to call before the subscription exists."""
    with _shared_pose_registry_lock:
        _shared_pose_listeners.setdefault(world_name, []).append(cb)


def unregister_pose_listener(world_name: str, cb) -> None:
    """Remove a callback previously registered with :func:`register_pose_listener`
    (a no-op if it is not registered)."""
    with _shared_pose_registry_lock:
        listeners = _shared_pose_listeners.get(world_name)
        if listeners and cb in listeners:
            listeners.remove(cb)


def _ensure_shared_pose_subscription(node, world_name: str) -> None:
    """Create the process-wide "/{world}/poses" subscription for ``world_name``
    (bound to ``node``) on first use; a no-op on subsequent calls."""
    with _shared_pose_registry_lock:
        if world_name in _shared_pose_subscriptions:
            return

        def _on_poses(msg: VesselPositionArray, _world=world_name):
            _shared_pose_tables[_world] = {
                vessel.vessel_name: vessel.pose for vessel in msg.vessels
            }
            _shared_pose_stamps[_world] = time.time()
            stamp = msg.header.stamp
            _shared_pose_sim_time[_world] = stamp.sec + stamp.nanosec * 1e-9
            # Fan out to pose-synced control loops. Snapshot under the lock so a
            # concurrent (de)registration can't corrupt the walk; isolate each
            # callback so one raising can't stop the others or pose updates.
            with _shared_pose_registry_lock:
                listeners = tuple(_shared_pose_listeners.get(_world, ()))
            for cb in listeners:
                try:
                    cb()
                except Exception:  # noqa: BLE001
                    logging.getLogger(__name__).exception(
                        "pose listener raised for world %s", _world
                    )

        _shared_pose_subscriptions[world_name] = node.create_subscription(
            VesselPositionArray,
            f"/{world_name}/poses",
            _on_poses,
            10,
            callback_group=_shared_callback_group,
        )


# Module-level registry of live Entity instances by world + agent_name, and ONE
# shared discovery timer per world — same consolidation as the MASCmd client
# and pose subscription above.
#
# Previously EVERY entity ran its OWN 1 Hz timer that called the graph-wide
# get_topic_names_and_types() RCL query AND linearly scanned the ENTIRE result
# for topics under its own name. With N entities in one process that's N
# separate expensive graph queries plus an O(N * total_topics) scan, every
# second — and the per-entity timer only self-cancels once THAT entity's own
# topics are fully discovered, which requires it to already be spawned. Agents
# still waiting on spawn confirmation therefore keep polling forever, adding
# load exactly while the process is already saturated confirming spawns — a
# feedback loop. A single shared timer queries the graph once per second and
# dispatches each matching topic to the right entity by name.
_entity_registry: dict[str, dict[str, "Entity"]] = {}
_entity_registry_lock = threading.Lock()
_shared_discovery_timers: dict[str, object] = {}


def _register_entity(world_name: str, agent_name: str, entity: "Entity") -> None:
    with _entity_registry_lock:
        _entity_registry.setdefault(world_name, {})[agent_name] = entity


def _rename_entity(world_name: str, old_name: str, new_name: str, entity: "Entity") -> None:
    with _entity_registry_lock:
        table = _entity_registry.setdefault(world_name, {})
        if table.get(old_name) is entity:
            del table[old_name]
        table[new_name] = entity


def _shared_discovery_tick(node, world_name: str) -> None:
    prefix = f"/{world_name}/"
    for topic_name, types in node.get_topic_names_and_types():
        if not topic_name.startswith(prefix):
            continue
        agent_name = topic_name[len(prefix):].split("/", 1)[0]
        with _entity_registry_lock:
            entity = _entity_registry.get(world_name, {}).get(agent_name)
        if entity is None or topic_name in entity._subscribed_topics:
            continue
        entity._subscribe_to_topic(topic_name, types)


def _ensure_shared_discovery_timer(node, world_name: str) -> None:
    with _entity_registry_lock:
        if world_name in _shared_discovery_timers:
            return
        _shared_discovery_timers[world_name] = node.create_timer(
            1.0,
            lambda: _shared_discovery_tick(node, world_name),
            callback_group=_shared_callback_group,
        )


class Entity(Agent):
    """
    Abstract base for agents that have a physical SDF model in the simulation world.

    Handles: SDF model, pose tracking, MAS spawn/delete, dynamic sensor subscription,
    and the concrete lotus_param() implementation shared by all physical entities.
    """

    def __init__(self, sdf_string: str, world_name: str, xdyn_port: int | None):
        self.num = self.get_unique_model_num()
        self.agent_name = f"{self.__class__.__name__.lower()}{self.num}"
        # The rclpy NODE name must be unique across the whole ROS graph: two
        # machines (or two processes) spawning an agent of the same class would
        # otherwise both register a node called e.g. "mybluerov0", which ROS 2
        # only tolerates with warnings and undefined behaviour. The node name is
        # purely a graph identifier here — all data routing goes through
        # agent_name (topics, poses, MAS cmds) — so we append a short random
        # suffix to the node name only, leaving agent_name as the clean logical
        # name the host deconflicts and the client adopts.
        node_name = f"{self.agent_name}_{uuid.uuid4().hex[:8]}"
        super().__init__(node_name, world_name)

        self.sdf_string = sdf_string.format(
            name=self.agent_name,
            port=xdyn_port if xdyn_port is not None else 0,
            world_name=world_name,
        )

        self.model_name: str = getattr(self, "model_name", "")
        self.renderer_type_name: str = getattr(self, "renderer_type_name", "")
        self.domains: list = getattr(self, "domains", [])
        self.thrusters: list = getattr(self, "thrusters", [])
        self.xdyn_ip: str | None = getattr(self, "xdyn_ip", None)
        self.xdyn_port: int | None = getattr(self, "xdyn_port", None)
        self.sdf_file: str = getattr(self, "sdf_file", "")

        # True once the host has confirmed THIS agent's own CREATE_CMD and we have
        # adopted the name it assigned (see confirm_spawn / send_single_mas_cmd_*).
        # missions_ready() gates the first mission tick on it so a leaf never binds
        # its topics to a same-named entity that belonged to another spawn. Reuse
        # paths that skip the CREATE_CMD set it directly.
        self._spawn_confirmed = False
        # True once the host has accepted this agent's CREATE_CMD goal.
        # Distinguishes a slow spawn from a lost one in the retry watchdog.
        self._spawn_goal_accepted = False

        self.sensor_buffers = {}
        self.sensors_subscribers = []
        self._subscribed_topics = set()

        # Shared per-process topic discovery (see _ensure_shared_discovery_timer):
        # register this entity so the one shared timer can dispatch matching
        # topics to it instead of every entity polling the graph itself.
        _register_entity(world_name, self.agent_name, self)
        _ensure_shared_discovery_timer(self, world_name)

        # Shared per-process MASCmd client (see _get_shared_mas_client): all agents
        # in this process route their own CREATE/DELETE goals through one client so
        # rclpy never cross-routes results between same-action clients.
        self.mas_action_client = _get_shared_mas_client(self, world_name)

        # Shared per-process pose subscription (see _ensure_shared_pose_subscription):
        # current_pose/last_pose_update below read this agent's own entry out of the
        # shared table instead of each agent scanning the full vessel list itself.
        _ensure_shared_pose_subscription(self, world_name)

    # ------------------------------------------------------------------
    # agent_name is a property: renaming an entity (launcher "id"-based names,
    # host deconfliction in confirm_spawn) must also move its entry in the
    # per-process entity registry, otherwise the shared discovery timer keeps
    # looking up topics under the OLD name and this agent's sensor topics are
    # never subscribed. The setter makes every call site
    # (``agent.agent_name = ...``) go through the registry rename transparently.
    # ------------------------------------------------------------------
    @property
    def agent_name(self) -> str:
        return self._agent_name

    @agent_name.setter
    def agent_name(self, new_name: str) -> None:
        old_name = getattr(self, "_agent_name", None)
        self._agent_name = new_name
        if old_name and old_name != new_name and getattr(self, "world_name", None):
            _rename_entity(self.world_name, old_name, new_name, self)

    @property
    def current_pose(self):
        return _shared_pose_tables.get(self.world_name, {}).get(self.agent_name)

    def poses_of_others(self) -> dict:
        """Live pose of every other vessel in this entity's world, keyed by
        name (same ground-truth table current_pose reads, e.g. for a task
        that needs to check proximity to another entity — a fake sonar/
        obstacle check, for instance)."""
        table = _shared_pose_tables.get(self.world_name, {})
        return {name: pose for name, pose in table.items() if name != self.agent_name}

    @property
    def last_pose_update(self) -> float:
        return _shared_pose_stamps.get(self.world_name, 0.0)

    @property
    def current_pose_sim_time(self) -> float | None:
        """Simulation-time stamp (seconds) of the latest /<world>/poses message,
        or None before the first pose arrives. Control loops key their dt off
        this instead of the wall clock so their ramps stay correct when the sim
        runs at a real-time factor != 1 (see WaypointFollowerTask)."""
        return _shared_pose_sim_time.get(self.world_name)

    def missions_ready(self) -> bool:
        """Hold the first mission tick until this entity is actually present in
        the simulation: its CREATE_CMD confirmed by the host AND its pose received
        on ``/<world>/poses``. Until then a leaf could bind its topics to a pose
        that belonged to another spawn sharing our pre-deconfliction name."""
        return self._spawn_confirmed and self.current_pose is not None

    def get_first_domain(self):
        return self.domains[0]

    def lotus_param(self) -> str:
        return f"<lotus_param>{self._lotus_blocks()}\n</lotus_param>"

    def _lotus_blocks(self) -> str:
        return f"""
    <render_interface>
        <publish_render>true</publish_render>
        <renderer_type_name>{self.renderer_type_name}</renderer_type_name>
    </render_interface>"""

    def confirm_spawn(self, assigned_name: str) -> None:
        """Adopt the host-assigned entity name and mark the spawn confirmed.

        The host (entity_manager) is the single authority on entity names: if the
        requested name is already taken (e.g. another machine spawned an agent of
        the same class) it deconflicts to a unique name and returns the actual name
        in ``Result.name``. We adopt it into ``agent_name`` so every topic / pose /
        mission / delete routes to the real entity. ``current_pose`` reads the
        shared pose table by ``agent_name``, so adopting the new name naturally
        reads as "no pose yet" until a pose under that name arrives — a pose that
        matched the *old* name (possibly another machine's same-named entity)
        cannot make missions start before our own entity reports in.
        """
        if assigned_name and assigned_name != self.agent_name:
            self.get_logger().info(
                f"Host assigned '{assigned_name}' instead of '{self.agent_name}' "
                f"(name already taken); adopting it for all topics."
            )
        if assigned_name and assigned_name != self.agent_name:
            # The agent_name setter moves this entity's registry entry.
            self.agent_name = assigned_name
        self._spawn_confirmed = True

    def _attach_spawn_result_handler(self, goal_future) -> None:
        """Wire a CREATE_CMD goal future to confirm_spawn once the host replies.

        Shared by every spawn path so both launchers (run_agent on the remote,
        agents_manager on the host) adopt the assigned name and flip
        ``_spawn_confirmed`` without duplicating the logic.
        """
        if goal_future is None:
            return

        name = self.agent_name

        def _on_goal(gf):
            try:
                goal_handle = gf.result()
            except Exception:
                self.get_logger().error(f"{name}: goal future.result() raised:", exc_info=True)
                return
            if goal_handle is None or not goal_handle.accepted:
                self.get_logger().error(f"{name}: spawn REJECTED by host.")
                return

            # Host accepted the goal; the result may still be pending.
            self._spawn_goal_accepted = True

            def _on_result(rf):
                try:
                    assigned = rf.result().result.name
                except Exception:
                    self.get_logger().error(f"{name}: result future.result() raised:", exc_info=True)
                    return
                if not assigned or assigned == "error_cmd":
                    self.get_logger().error(f"{name}: host failed to spawn entity.")
                    return
                self.confirm_spawn(assigned)

            goal_handle.get_result_async().add_done_callback(_on_result)

        goal_future.add_done_callback(_on_goal)

    def send_single_mas_cmd(self, value, server_timeout_sec: float = 5.0):
        if isinstance(value, (list, tuple)):
            if len(value) == 2:
                lat, lon = value
                return self.send_single_mas_cmd_geo(lat, lon, 0.0, server_timeout_sec)
            elif len(value) == 3:
                lat, lon, alt = value
                return self.send_single_mas_cmd_geo(lat, lon, alt, server_timeout_sec)
            elif len(value) == 6:
                return self.send_single_mas_cmd_pose(value, server_timeout_sec)
        raise ValueError(
            "send_single_mas_cmd() requires [lat, lon], [lat, lon, alt], "
            "or [x, y, z, roll, pitch, yaw]"
        )

    @staticmethod
    def _pose6_to_pose_msg(pose) -> Pose:
        """Convert a 6-element ``[x, y, z, roll, pitch, yaw]`` into a geometry Pose
        (position + quaternion). Single source of truth for both the single-cmd and
        batch spawn paths."""
        pose = [float(v) for v in pose[:6]]
        roll, pitch, yaw = pose[3], pose[4], pose[5]
        cr, sr = math.cos(roll / 2), math.sin(roll / 2)
        cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
        cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
        pose_msg = Pose()
        pose_msg.position.x, pose_msg.position.y, pose_msg.position.z = pose[:3]
        pose_msg.orientation.w = cr * cp * cy + sr * sp * sy
        pose_msg.orientation.x = sr * cp * cy - cr * sp * sy
        pose_msg.orientation.y = cr * sp * cy + sr * cp * sy
        pose_msg.orientation.z = cr * cp * sy - sr * sp * cy
        return pose_msg

    def _build_create_cmd(self, value) -> MASCmd:
        """Build a CREATE_CMD ``MASCmd`` for THIS agent from a pose or lat/lon value.

        ``value`` is ``[x, y, z, roll, pitch, yaw]`` (ENU pose), ``[lat, lon]`` or
        ``[lat, lon, alt]`` (geographic). Shared by ``send_single_mas_cmd_*`` and the
        batch spawn helper so the payload is identical whichever path sends it."""
        cmd = MASCmd()
        cmd.cmd_type = MASCmd.CREATE_CMD
        cmd.model_name = self.model_name
        cmd.sdf_file = self.sdf_file
        cmd.vessel_name = self.agent_name
        cmd.sdf_string = self.lotus_param()
        if isinstance(value, (list, tuple)) and len(value) == 6:
            cmd.vessel_position = self._pose6_to_pose_msg(value)
        elif isinstance(value, (list, tuple)) and len(value) in (2, 3):
            geo = GeoPoint()
            geo.latitude = float(value[0])
            geo.longitude = float(value[1])
            geo.altitude = float(value[2]) if len(value) == 3 else 0.0
            cmd.geo_point = geo
        else:
            raise ValueError(
                "_build_create_cmd() requires [lat, lon], [lat, lon, alt], "
                "or [x, y, z, roll, pitch, yaw]"
            )
        return cmd

    def _arm_spawn_retry_watchdog(
        self, resend_fn, retries_left: int, timeout_sec: float = 30.0, patience_left: int = 10
    ) -> None:
        """Resend a CREATE_CMD only if its goal was never accepted.

        ``_spawn_goal_accepted`` distinguishes a lost request from a slow one:
        - accepted, not yet confirmed: wait for the result callback (bounded by
          ``patience_left``); resending would duplicate an in-flight spawn.
        - never accepted after ``timeout_sec``: treat as lost; delete by name and
          resend (bounded by ``retries_left``).
        """
        if retries_left <= 0:
            return

        def _check():
            timer.cancel()
            if self._spawn_confirmed:
                return
            if self._spawn_goal_accepted:
                # Accepted, not yet confirmed: wait for the result callback
                # rather than resend. Bounded by patience_left so a host that
                # dies after accepting cannot spin the timer forever.
                if patience_left > 0:
                    self._arm_spawn_retry_watchdog(
                        resend_fn, retries_left, timeout_sec, patience_left - 1
                    )
                return
            self.get_logger().warning(
                f"{self.agent_name}: spawn goal not accepted after {timeout_sec}s, "
                f"resending CREATE_CMD ({retries_left} retries left)."
            )
            # Never accepted: treat as lost. Delete by name so the resend leaves
            # no ghost entity at the spawn point.
            self.send_single_delete_cmd()
            resend_fn()

        timer = self.create_timer(timeout_sec, _check)

    def send_single_mas_cmd_geo(
        self, lat, lon, alt=0.0, server_timeout_sec: float = 5.0, _retries_left: int = 2
    ):
        goal_msg = lotusim_msgs.action.MASCmd.Goal()
        goal_msg.cmd = self._build_create_cmd([lat, lon, alt])

        self.get_logger().info(f"Sending MAS command with GeoPoint: lat={lat}, lon={lon}, alt={alt}")

        if not self.mas_action_client.wait_for_server(timeout_sec=server_timeout_sec):
            self.get_logger().error(f"{self.agent_name}: MASCmd server unavailable.")
            return None
        goal_future = self.mas_action_client.send_goal_async(goal_msg)
        self._attach_spawn_result_handler(goal_future)
        self._arm_spawn_retry_watchdog(
            lambda: self.send_single_mas_cmd_geo(lat, lon, alt, server_timeout_sec, _retries_left - 1),
            _retries_left,
        )
        return goal_future

    def send_single_mas_cmd_pose(self, pose, server_timeout_sec: float = 5.0, _retries_left: int = 2):
        goal_msg = lotusim_msgs.action.MASCmd.Goal()
        goal_msg.cmd = self._build_create_cmd(list(pose[:6]))

        self.get_logger().info(f"Sending MAS command with XYZ pose: {list(pose[:6])}")

        if not self.mas_action_client.wait_for_server(timeout_sec=server_timeout_sec):
            self.get_logger().error(f"{self.agent_name}: MASCmd server unavailable.")
            return None
        goal_future = self.mas_action_client.send_goal_async(goal_msg)
        self._attach_spawn_result_handler(goal_future)
        self._arm_spawn_retry_watchdog(
            lambda: self.send_single_mas_cmd_pose(pose, server_timeout_sec, _retries_left - 1),
            _retries_left,
        )
        return goal_future

    def send_single_delete_cmd(self, server_timeout_sec: float = 5.0):
        goal_msg = lotusim_msgs.action.MASCmd.Goal()
        cmd = MASCmd()
        cmd.cmd_type = MASCmd.DELETE_CMD
        cmd.vessel_name = self.agent_name
        goal_msg.cmd = cmd

        if not self.mas_action_client.wait_for_server(timeout_sec=server_timeout_sec):
            self.get_logger().error(f"{self.agent_name}: MASCmd server unavailable.")
            return None
        return self.mas_action_client.send_goal_async(goal_msg)

    def _subscribe_to_topic(self, topic_name: str, types: list[str]) -> None:
        """Subscribe to one of THIS entity's own topics, found by the shared
        per-process discovery timer (see _shared_discovery_tick)."""
        type_name = types[0] if types else "std_msgs/msg/String"
        try:
            MsgType = get_message(type_name)
        except Exception:
            # Expected on the remote for sensor types whose message package (e.g.
            # lotusim_sensor_msgs, host-only) isn't part of the deployment bundle.
            # Nothing currently reads sensor_buffers for these topics, so the
            # String fallback is harmless — debug level keeps it out of the
            # console without hiding it entirely.
            self.get_logger().debug(
                f"Cannot load message type {type_name} for topic {topic_name}, using String fallback"
            )
            MsgType = String

        buffer_name = topic_name.split("/")[-1].lower()
        sub = self.create_subscription(
            MsgType,
            topic_name,
            lambda msg, b=buffer_name, t=topic_name: self._sensor_callback(msg, b, t),
            self.qos_profile,
        )
        self.sensors_subscribers.append(sub)
        self._subscribed_topics.add(topic_name)

    def _sensor_callback(self, msg, buffer_name: str, topic_name: str):
        buffer = self.sensor_buffers.setdefault(buffer_name, [])
        buffer.append(msg)
        if len(buffer) > 100:
            buffer.pop(0)

    def start_pause(self, duration: float):
        self.timer = self.create_timer(duration, self.resume_agent)

    def resume_agent(self):
        if hasattr(self, "timer") and self.timer:
            self.timer.cancel()


# Concrete physical and fixed entity agents
from lotusim_sdk.agents.entity.physical import (  # noqa: E402
    Bluerov2Heavy,
    Commando,
    DtmbHull,
    Fremm,
    Lrauv,
    Mine,
    Pha,
    Wamv,
    X500,
)

__all__ = [
    "Entity",
    "_get_shared_mas_client",
    "_get_shared_mas_array_client",
    "send_batch_mas_cmd",
    "Bluerov2Heavy",
    "Commando",
    "DtmbHull",
    "Fremm",
    "Lrauv",
    "Mine",
    "Pha",
    "Wamv",
    "X500",
]
