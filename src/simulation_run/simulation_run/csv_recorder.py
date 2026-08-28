"""
@file csv_recorder.py
@brief Observer node recording every agent of a LOTUSim world to CSV files.

A pure observer — it spawns NOTHING in the simulation and is not tied to any
agent, so it fits the distributed multi-agent architecture: the recording is a
property of whoever wants the data (host or any machine on the ROS network),
never of the agents themselves.

Enable it from the scenario JSON (host side)::

    { "record_csv": true, ... }
    { "record_csv": {"rate": 5.0, "outdir": "my_csv_dir", "prefix": "run1_"}, ... }
    {
      "record_csv": {
        "rate": 5.0,
        "ref_lat": 50.32879166666667,
        "ref_lon": -4.195226666666667,
        "ref_alt": 0.0
      },
      ...
    }

or run it standalone from anywhere (see ``scripts/log_run_csv.py``).

It subscribes to ``/<world>/poses`` (authoritative host-side poses of ALL
agents) and auto-discovers every ``/<world>/<agent>/battery/state`` topic
(published for models spawned with a battery sensor, e.g.
``model-battery.sdf``). Simulation time comes from the Gazebo ``/stats`` topic
when the gz Python bindings are available on this machine (host case);
otherwise it falls back to wall-clock seconds since the recorder started.

Latitude/longitude are derived from the local ENU ``pos_x``/``pos_y`` using a
WGS84 local tangent-plane conversion about a reference point (``ref_lat``,
``ref_lon``, ``ref_alt``) — the same origin as the world's
``<spherical_coordinates>`` block. This mirrors what gz-math's
``SphericalCoordinates`` (``LOCAL2``) does for small-scale scenes, but is
implemented locally so it works the same whether or not the gz Python
bindings are installed on this machine (host or remote).

If the full scenario dict is passed to :func:`recorder_from_config` (so the
mission list is known), the recorder also reconstructs, per agent, the
*intended* straight-line path for each mission in order (spawn -> mission 1's
waypoints -> mission 2's waypoints -> ...) and compares the *actual*
simulated position against it every sample. This is meant for
current/disturbance studies: it separates "where they meant to go" from
"where they actually ended up", and also tells you which task is currently
active, without needing any change to the waypoint_follower plugin itself.
Concretely, per sample:

- ``mission_id`` / ``task_type`` — the ``id``/``task`` of whichever mission
  in the agent's ``missions`` list is currently active.
- ``distance_to_target_m`` — straight-line distance to the waypoint currently
  being pursued (only for ``waypoint_follower`` missions).
- ``cross_track_error_m`` — perpendicular distance from the actual position
  to the *planned* straight line for the current leg. This is the drift
  caused by a current.
- ``along_track_progress_pct`` — progress (0-100%) along the current leg,
  measured along the planned line (not affected by the current).
- ``waypoint_arrived`` / ``arrival_error_m`` / ``arrival_error_pct`` — set on
  the sample where the agent comes within that mission's ``range_tolerance``
  of the target waypoint (the same tolerance the waypoint_follower task uses
  to decide arrival). ``arrival_error_pct`` expresses that arrival distance
  as a percentage of the leg length, so it's comparable across legs of
  different sizes. Once the last waypoint of a mission is reached, the
  recorder automatically advances to the next mission in the list.
- ``mission_complete`` — 1 once the agent has arrived at the last waypoint
  of its *last* mission.

Caveats:

- Only ``waypoint_follower`` missions can be progress-tracked (distance,
  cross-track, arrival) because they're the only task type with an explicit,
  known target. Missions of any other ``task`` still show up in
  ``mission_id``/``task_type`` while active, but the recorder has no signal
  for when they finish — it can't auto-advance past one, so any missions
  after a non-waypoint_follower one in the same agent's list won't be
  reached. If your agents mix task types, treat this as a "task label" for
  the currently-known mission rather than full sequencing.
- Waypoints within a mission are assumed visited strictly in order with no
  looping (matches ``"loop": false``).
- Assumes the ROS ``agent_name`` on ``/<world>/poses`` matches the agent
  ``"id"`` in the scenario JSON.

One CSV file **per agent** (``<outdir>/<prefix><agent>.csv``). The header is
NOT the same for every agent: each column GROUP is only included for an
agent that actually has the thing it describes, decided from the scenario
config (not from what happens to arrive first at runtime):

- Base columns (every agent): ``agent_name, sim_time_s, pos_x, pos_y,
  pos_z, lat, lon, orient_x, orient_y, orient_z, orient_w,
  current_vx_mps, current_vy_mps, current_vz_mps`` — the last three are the
  configured ``OceanCurrent`` agent (see
  ``lotusim_sdk.agents.environment.ocean_current``), repeated on every row so
  a single agent's file is self-contained for "what current was this agent
  under." ``current_vz_mps`` reflects the agent's configured ``z``, but
  KinematicInterface's own pose integration is 2D (x/y + yaw) only, see
  §5.4 of ``WRITE_SCENARIO.md`` — it has no physical effect yet.
- Battery columns (only agents spawned with a battery sensor, e.g.
  ``sdf_file: "model-battery.sdf"``): ``battery_voltage, battery_charge_ah,
  battery_capacity_ah, battery_percentage, battery_status``.
- Mission columns (only agents with a real navigation mission — any task
  other than the purely-technical ``kinematic_anchor``, see
  ``lotusim_sdk.tasks.kinematic_anchor``): ``mission_id, task_type,
  target_waypoint_idx, target_x, target_y, target_lat, target_lon,
  distance_to_target_m, cross_track_error_m, along_track_progress_pct,
  waypoint_arrived, arrival_error_m, arrival_error_pct, mission_complete``.
- Sonar columns (only agents whose mission sets ``params.sonar_range_m``,
  e.g. ``waypoint_follower_avoidance``): ``sonar_range_m, sonar_contact,
  sonar_distance_m`` — a FAKE sonar (ground-truth proximity to every agent
  whose scenario ``"class"`` is ``"mine"``), not a simulated acoustic
  sensor. Deliberately NO identity column: range only, like a real sonar —
  the recorder uses the nearest object's true name internally (to pick the
  closest one and to count distinct contacts for summary.csv) but never
  writes it out here.

A mine, for example, gets ONLY the base columns — no battery, no mission, no
sonar — because it has none of those; it doesn't need a mission to have its
(current-drifted) position logged.

A ``summary.csv`` (one row per agent) is written once when the recorder shuts
down: the agent's ``sensors``/``actuators`` equipment manifest (see
:func:`_agent_capabilities_from_scenario` — a ``;``-joined list, e.g.
``"battery;sonar"``, empty for an unequipped prop like a mine), objective
(spawn -> final target lat/lon), outcome (``mission_complete``/final BT
status if known), final arrival error, and difficulty indicators —
``max_cross_track_error_m``, ``min_sonar_distance_m``,
``distinct_obstacles_detected`` (a COUNT, not a list of names — same "no
identity" rule as the per-sample sonar columns) and ``time_in_sonar_contact_s``
(how close it got to an obstacle and how long avoidance was active), plus
battery start/end and the world origin/current repeated as global columns.
Meant as a compact, numeric per-run fact sheet — e.g. for an LLM to turn
into a narrative report — not prose itself.

Rows are flushed continuously so a hard kill loses nothing (except the final
``summary.csv`` — that one is only written on clean shutdown).
"""

import csv
import math
import os
import threading
import time

from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import BatteryState

from lotusim_msgs.msg import VesselPositionArray

# sensor_msgs/BatteryState power_supply_status values.
_STATUS_MAP = {
    0: "UNKNOWN",
    1: "CHARGING",
    2: "DISCHARGING",
    3: "NOT_CHARGING",
    4: "FULL",
}

# Every agent gets these — pose is the one thing every physical entity has
# regardless of sensors/mission.
_BASE_HEADER = [
    "agent_name",
    "sim_time_s",
    "pos_x", "pos_y", "pos_z",
    "lat", "lon",
    "orient_x", "orient_y", "orient_z", "orient_w",
    "current_vx_mps", "current_vy_mps", "current_vz_mps",
]
# Only for agents spawned with a battery sensor (sdf_file containing
# "battery", e.g. "model-battery.sdf") — a mine on the plain model.sdf has no
# battery, so it gets no battery_* columns at all rather than blank/zero ones.
_BATTERY_HEADER = [
    "battery_voltage",
    "battery_charge_ah",
    "battery_capacity_ah",
    "battery_percentage",
    "battery_status",
]
# Only for agents with a real navigation mission (any task other than the
# purely technical "kinematic_anchor" — see lotusim_sdk.tasks.kinematic_anchor).
# A mine "knows its position" from the ground-truth pose table like everyone
# else; it doesn't need a mission for that, so it gets no mission_* columns.
_MISSION_HEADER = [
    "mission_id",
    "task_type",
    "target_waypoint_idx",
    "target_x", "target_y",
    "target_lat", "target_lon",
    "distance_to_target_m",
    "cross_track_error_m",
    "along_track_progress_pct",
    "waypoint_arrived",
    "arrival_error_m",
    "arrival_error_pct",
    "mission_complete",
]
# Only for agents whose mission actually configures a sonar_range_m (e.g.
# waypoint_follower_avoidance) — a plain waypoint_follower vehicle or a mine
# has no sonar, so it gets no sonar_* columns. Deliberately NO identity
# column (no "sonar_target") — a real sonar reports range, not "this is
# mine0 vs mine1"; recording an object's true identity here would leak
# ground truth the vehicle's own sensor could never have.
_SONAR_HEADER = [
    "sonar_range_m",
    "sonar_contact",
    "sonar_distance_m",
]

# WGS84 ellipsoid constants (same values gz-math's SphericalCoordinates uses).
_WGS84_A = 6378137.0            # semi-major axis, m
_WGS84_E2 = 6.69437999014e-3    # first eccentricity squared

# Default reference origin: matches energy.world's <spherical_coordinates>.
_DEFAULT_REF_LAT = 50.32879166666667
_DEFAULT_REF_LON = -4.195226666666667
_DEFAULT_REF_ALT = 0.0


def _enu_to_latlon(x: float, y: float, ref_lat_deg: float, ref_lon_deg: float,
                    ref_alt: float = 0.0) -> tuple:
    """Convert a local ENU offset (x=East, y=North, metres) to (lat, lon) in
    degrees, using a WGS84 local tangent-plane approximation about
    (ref_lat_deg, ref_lon_deg, ref_alt).

    This is the same radii-of-curvature approach gz-math's
    SphericalCoordinates uses for LOCAL2 <-> SPHERICAL conversions, and is
    accurate to sub-metre level over scenes spanning a few kilometres.
    """
    lat0 = math.radians(ref_lat_deg)
    sin_lat0 = math.sin(lat0)
    denom = 1.0 - _WGS84_E2 * sin_lat0 * sin_lat0

    # Prime vertical (east-west) radius of curvature.
    n_radius = _WGS84_A / math.sqrt(denom)
    # Meridian (north-south) radius of curvature.
    m_radius = _WGS84_A * (1.0 - _WGS84_E2) / (denom ** 1.5)

    dlat = y / (m_radius + ref_alt)
    dlon = x / ((n_radius + ref_alt) * math.cos(lat0))

    lat = ref_lat_deg + math.degrees(dlat)
    lon = ref_lon_deg + math.degrees(dlon)
    return lat, lon


def _latlon_to_enu(lat_deg: float, lon_deg: float, ref_lat_deg: float,
                    ref_lon_deg: float, ref_alt: float = 0.0) -> tuple:
    """Inverse of :func:`_enu_to_latlon`: (lat, lon) degrees -> local ENU
    (x=East, y=North) metres about the same reference point/approximation.
    """
    lat0 = math.radians(ref_lat_deg)
    sin_lat0 = math.sin(lat0)
    denom = 1.0 - _WGS84_E2 * sin_lat0 * sin_lat0

    n_radius = _WGS84_A / math.sqrt(denom)
    m_radius = _WGS84_A * (1.0 - _WGS84_E2) / (denom ** 1.5)

    dlat = math.radians(lat_deg - ref_lat_deg)
    dlon = math.radians(lon_deg - ref_lon_deg)

    y = dlat * (m_radius + ref_alt)
    x = dlon * (n_radius + ref_alt) * math.cos(lat0)
    return x, y


def _leg_geometry(prev_xy: tuple, target_xy: tuple, pos_xy: tuple) -> tuple:
    """Return (distance_to_target_m, cross_track_error_m, along_track_pct)
    for ``pos_xy`` relative to the planned straight line from ``prev_xy`` to
    ``target_xy``.

    ``cross_track_error_m`` is the unsigned perpendicular distance from the
    actual position to that planned line — i.e. how far a current has pushed
    the agent off its intended track. ``along_track_pct`` is progress along
    the line (clamped to [0, 100]), independent of any cross-track drift.
    """
    px, py = prev_xy
    tx, ty = target_xy
    x, y = pos_xy

    vx, vy = tx - px, ty - py
    leg_len = math.hypot(vx, vy)
    distance_to_target = math.hypot(x - tx, y - ty)

    if leg_len < 1e-6:
        return distance_to_target, 0.0, 100.0

    wx, wy = x - px, y - py
    t = (wx * vx + wy * vy) / (leg_len * leg_len)
    cross_track = abs(wx * vy - wy * vx) / leg_len
    progress_pct = max(0.0, min(1.0, t)) * 100.0
    return distance_to_target, cross_track, progress_pct


def _mission_specs_from_scenario(scenario: dict, ref_lat: float, ref_lon: float,
                                  ref_alt: float) -> dict:
    """Build the ordered list of mission specs for every agent in a parsed
    scenario JSON dict.

    Each spec is ``{"id", "task", "path", "tolerance", "sonar_range_m"}``.
    ``path`` is the ENU point list ``[leg_start, waypoint1, waypoint2, ...]``
    for any mission whose params include a ``waypoints`` list — regardless of
    the exact task name, so a subclass of ``waypoint_follower`` (e.g.
    ``waypoint_follower_avoidance``, same params shape) is tracked exactly
    the same way — and ``None`` for any other task type (nothing to track
    geometrically, but the id/task are still reported while it's the active
    mission). ``sonar_range_m`` is the mission's ``params.sonar_range_m`` if
    present (0.0 otherwise), used to gate the CSV sonar columns.

    Returns a dict keyed by the actual SPAWNED INSTANCE name — ``f"{id}{i}"``,
    the same ``agents_manager.py`` naming used for every agent's entry on
    ``/<world>/poses`` (confirmed by e.g. a ``nb_agents: 1`` agent with
    ``"id": "bluerov1"`` spawning as agent ``"bluerov10"``, NOT the bare
    ``"bluerov1"``) — NOT the bare scenario ``"id"``, so ``_mission_columns``'s
    ``self._mission_specs.get(agent)`` lookup (keyed by that same live
    agent name) actually matches. Agents with no missions are omitted.
    """
    specs = {}
    for agent in scenario.get("agents", []):
        id_base = agent.get("id") or ""
        spawn = agent.get("spawn", {})
        base_cursor = (float(spawn.get("x", 0.0)), float(spawn.get("y", 0.0)))
        mission_list = []
        cursor = base_cursor
        for mission in agent.get("missions", []):
            params = mission.get("params", {}) or {}
            entry = {
                "id": mission.get("id", ""),
                "task": mission.get("task", mission.get("type", "")),
                "path": None,
                "tolerance": 3.0,
                "sonar_range_m": float(params.get("sonar_range_m", 0.0)),
            }
            waypoints = params.get("waypoints")
            if waypoints:
                entry["tolerance"] = float(params.get("range_tolerance", 3.0))
                path = [cursor]
                for wp in waypoints:
                    if "x" in wp and "y" in wp:
                        path.append((float(wp["x"]), float(wp["y"])))
                    else:
                        path.append(
                            _latlon_to_enu(wp["lat"], wp["lon"], ref_lat, ref_lon, ref_alt)
                        )
                entry["path"] = path
                cursor = path[-1]  # next mission's leg starts where this one ended
            mission_list.append(entry)
        if not mission_list:
            continue
        for i in range(int(agent.get("nb_agents", 1))):
            specs[f"{id_base}{i}"] = mission_list
    return specs


def _obstacle_agents_from_scenario(scenario: dict) -> set:
    """Instance names of every agent whose class is ``"mine"`` (case-
    insensitive), expanded with the same ``f"{id}{index}"`` naming
    ``agents_manager.py`` uses to name spawned instances. Used to gate the
    CSV sonar columns and find each agent's nearest obstacle."""
    obstacles = set()
    for agent in scenario.get("agents", []):
        agent_class = agent.get("class") or agent.get("type") or agent.get("id") or ""
        if str(agent_class).lower() != "mine":
            continue
        id_base = agent.get("id") or str(agent_class).lower()
        for i in range(int(agent.get("nb_agents", 1))):
            obstacles.add(f"{id_base}{i}")
    return obstacles


# Utility BT tasks that force a connection type / do bookkeeping but are not
# a real navigation mission (see lotusim_sdk.tasks.kinematic_anchor) — an
# agent whose entire mission list is made of these gets no mission_*
# columns in its CSV, same treatment as having no missions at all.
_NON_NAVIGATION_TASKS = {"kinematic_anchor"}


def _ocean_current_from_scenario(scenario: dict) -> tuple:
    """Initial ``(x, y, z)`` m/s (ENU) of the scenario's ``OceanCurrent``
    agent, if any — read straight from its JSON block in ``"agents"`` (the
    same static-config value the agent is constructed with), since a CSV
    reader has no live handle to the running agent to pick up later
    ``set_current`` mission changes."""
    for agent in scenario.get("agents", []):
        agent_class = agent.get("class") or agent.get("type") or agent.get("id") or ""
        if str(agent_class).lower() != "oceancurrent":
            continue
        return (
            float(agent.get("x", 0.0)),
            float(agent.get("y", 0.0)),
            float(agent.get("z", 0.0)),
        )
    return (0.0, 0.0, 0.0)


def _agent_capabilities_from_scenario(scenario: dict) -> dict:
    """Per spawned-instance-name capability flags used to decide which
    column groups (§ ``_BATTERY_HEADER``/``_MISSION_HEADER``/
    ``_SONAR_HEADER``) an agent's CSV actually gets — driven entirely by the
    scenario config (known upfront, no live-discovery race), not by
    whatever happens to arrive first at runtime.

    An agent's JSON block may declare an explicit equipment manifest —
    ``"sensors": ["battery", "sonar"]``, ``"actuators": ["thrusters"]`` —
    which is authoritative when present (and is what ends up in
    ``summary.csv``'s ``sensors``/``actuators`` columns). Where an agent
    doesn't declare it (older scenario files), battery/sonar fall back to
    the pre-existing heuristics (``sdf_file`` naming, mission
    ``params.sonar_range_m``) and ``sensors``/``actuators`` are reconstructed
    from those same booleans, so nothing breaks for scenarios that predate
    this field. This is deliberately a LOGGING-side manifest only — it does
    not change how an agent is actually spawned/equipped (that's still
    ``sdf_file`` + mission ``task``/``params``, unchanged); declaring
    ``"sonar"`` here without also giving the agent a
    ``waypoint_follower_avoidance`` mission would just make the CSV lie.

    Returns ``{agent_name: {"battery": bool, "mission": bool, "sonar": bool,
    "sensors": [...], "actuators": [...]}}``.
    """
    caps = {}
    for agent in scenario.get("agents", []):
        id_base = agent.get("id") or ""
        missions = agent.get("missions") or []
        declared_sensors = agent.get("sensors")
        declared_actuators = agent.get("actuators")

        has_battery = (
            "battery" in declared_sensors if declared_sensors is not None
            else "battery" in str(agent.get("sdf_file", "")).lower()
        )
        has_mission = any(
            m.get("task", m.get("type", "")) not in _NON_NAVIGATION_TASKS
            for m in missions
        )
        has_sonar = (
            "sonar" in declared_sensors if declared_sensors is not None
            else any(
                float((m.get("params") or {}).get("sonar_range_m", 0.0)) > 0.0
                for m in missions
            )
        )
        sensors = (
            list(declared_sensors) if declared_sensors is not None
            else (["battery"] if has_battery else []) + (["sonar"] if has_sonar else [])
        )
        actuators = (
            list(declared_actuators) if declared_actuators is not None
            else (["thrusters"] if has_mission else [])
        )
        entry = {
            "battery": has_battery, "mission": has_mission, "sonar": has_sonar,
            "sensors": sensors, "actuators": actuators,
        }
        for i in range(int(agent.get("nb_agents", 1))):
            caps[f"{id_base}{i}"] = entry
    return caps


class CsvRecorder(Node):
    """Record all agents of ``world`` to one CSV per agent in ``outdir``."""

    def __init__(
        self,
        world: str,
        outdir: str,
        prefix: str = "",
        rate_hz: float = 2.0,
        ref_lat: float = _DEFAULT_REF_LAT,
        ref_lon: float = _DEFAULT_REF_LON,
        ref_alt: float = _DEFAULT_REF_ALT,
        mission_specs: dict = None,
        obstacle_agents: set = None,
        agent_capabilities: dict = None,
        ocean_current: tuple = (0.0, 0.0, 0.0),
    ) -> None:
        super().__init__("csv_recorder")
        self._world = world
        self._outdir = outdir
        self._prefix = prefix
        self._ref_lat = ref_lat
        self._ref_lon = ref_lon
        self._ref_alt = ref_alt
        self._current_vx, self._current_vy, self._current_vz = ocean_current
        # agent -> [ {id, task, path, tolerance, sonar_range_m}, ... ] ordered mission list
        self._mission_specs = mission_specs or {}
        # agent names whose scenario "class" is "mine" — the fake sonar's targets.
        self._obstacle_agents = obstacle_agents or set()
        # agent -> {"battery","mission","sonar"} bool flags, from the scenario
        # config — decides which column groups that agent's CSV gets at all.
        # Unknown agent (e.g. no scenario passed): default to the pre-this-
        # feature behaviour (battery + mission columns present, blank/0 if
        # not applicable; sonar opt-in only, it's new and needs config).
        self._agent_capabilities = agent_capabilities or {}
        self._default_capabilities = {
            "battery": True, "mission": True, "sonar": False,
            "sensors": [], "actuators": [],
        }
        # agent -> index into _mission_specs[agent] of the currently active mission
        self._mission_idx = {v: 0 for v in self._mission_specs}
        # agent -> index into that mission's path of the current target waypoint
        self._leg_idx = {v: 1 for v in self._mission_specs}
        # agent -> minimum distance-to-target ever seen on the CURRENT leg,
        # reset on every leg/mission advance. Arrival is decided from this,
        # not the latest sample's distance: an agent that stops at its
        # target keeps drifting under the current afterward (nothing left to
        # correct it — same as an unpowered mine), so by the time a later
        # sample is taken it can legitimately be back outside tolerance even
        # though it genuinely reached the target earlier. "Did it ever get
        # close enough" is the right question, not "is it still there now".
        self._leg_min_distance = {v: float("inf") for v in self._mission_specs}
        self._all_missions_complete = {v: False for v in self._mission_specs}
        # Agents whose _all_missions_complete can ever become True: at least
        # one mission with a real waypoint path. An agent with only
        # path-less missions (e.g. a mine on "kinematic_anchor", which runs
        # forever by design — see KinematicAnchorTask) never sets its flag in
        # _mission_columns(), so it must be excluded here too, or the early-
        # stop check below would wait forever for something that can't finish.
        self._trackable_agents = {
            v for v, specs in self._mission_specs.items()
            if any(m["path"] for m in specs)
        }
        # Once every tracked agent has arrived, recording stops itself (timers
        # cancelled, summary written) even though the rest of the simulation —
        # Gazebo world, other agents, anything launched later — keeps running.
        # This node isn't destroyed, just made inert; destroy_node() re-uses
        # the same finalize path (idempotent) for the ordinary full-shutdown case.
        self._finalized = False
        self._poses = {}          # agent_name -> Pose (latest)
        self._battery = {}        # agent_name -> dict (latest reading)
        self._battery_subs = {}   # topic -> subscription
        self._files = {}          # agent_name -> (file, csv.writer)
        # agent -> running stats for summary.csv (difficulty indicators etc.),
        # lazily created per agent in _stats_for().
        self._stats = {}
        self._t0 = time.time()

        os.makedirs(outdir, exist_ok=True)

        # Simulation time from Gazebo /stats (host machine only — gz transport
        # does not cross machines the way ROS does). Falls back to wall clock.
        self._sim_time = None
        self._sim_time_lock = threading.Lock()
        self._gz_node = None
        try:
            from gz.transport13 import Node as GzNode
            from gz.msgs10.world_stats_pb2 import WorldStatistics

            def _on_stats(msg: WorldStatistics) -> None:
                with self._sim_time_lock:
                    self._sim_time = msg.sim_time.sec + msg.sim_time.nsec * 1e-9

            self._gz_node = GzNode()
            self._gz_node.subscribe(WorldStatistics, "/stats", _on_stats)
            self.get_logger().info("Sim time source: gz /stats")
        except Exception as e:  # bindings absent or remote machine
            self.get_logger().warning(
                f"gz /stats unavailable ({e}); sim_time_s falls back to wall clock."
            )

        self._rate_hz = rate_hz  # for stats["contact_samples"] -> seconds in summary.csv

        self.create_subscription(
            VesselPositionArray, f"/{world}/poses", self._on_poses, 10
        )
        # Battery topics only appear once their agent is spawned: poll the graph.
        self._battery_timer = self.create_timer(1.0, self._discover_battery_topics)
        self._sample_timer = self.create_timer(1.0 / rate_hz, self._sample)
        self.get_logger().info(
            f"Recording /{world}/poses (+batteries) at {rate_hz} Hz, "
            f"one CSV per agent in {outdir}/ "
            f"(lat/lon origin: {ref_lat:.8f}, {ref_lon:.8f}); "
            f"mission tracking for: {sorted(self._mission_specs) or 'none'}; "
            f"sonar obstacles: {sorted(self._obstacle_agents) or 'none'}"
        )

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------
    def _on_poses(self, msg: VesselPositionArray) -> None:
        self._poses = {v.vessel_name: v.pose for v in msg.vessels}

    def _on_battery(self, agent: str, msg: BatteryState) -> None:
        self._battery[agent] = {
            "voltage": msg.voltage,
            "charge": msg.charge,
            "capacity": msg.capacity,
            "percentage": msg.percentage,
            "status": _STATUS_MAP.get(msg.power_supply_status, "UNKNOWN"),
        }

    def _discover_battery_topics(self) -> None:
        prefix = f"/{self._world}/"
        for topic, _types in self.get_topic_names_and_types():
            if not topic.startswith(prefix) or not topic.endswith("/battery/state"):
                continue
            if topic in self._battery_subs:
                continue
            agent = topic[len(prefix):].split("/", 1)[0]
            # Match the battery_sensor publisher QoS (TRANSIENT_LOCAL) so the
            # latest reading arrives even if it was published before this
            # subscription was created.
            self._battery_subs[topic] = self.create_subscription(
                BatteryState,
                topic,
                lambda msg, v=agent: self._on_battery(v, msg),
                QoSProfile(
                    depth=10,
                    reliability=ReliabilityPolicy.RELIABLE,
                    durability=DurabilityPolicy.TRANSIENT_LOCAL,
                ),
            )
            self.get_logger().info(f"Found battery topic for '{agent}'")

    # ------------------------------------------------------------------
    # Recording
    # ------------------------------------------------------------------
    def _capabilities(self, agent: str) -> dict:
        return self._agent_capabilities.get(agent, self._default_capabilities)

    def _stats_for(self, agent: str) -> dict:
        """Running per-agent stats accumulated sample-by-sample, written out
        as difficulty/outcome indicators in summary.csv on shutdown."""
        stats = self._stats.get(agent)
        if stats is None:
            stats = {
                "max_cross_track_m": 0.0,
                "min_sonar_distance_m": None,
                # Ground-truth object identities, used only to count distinct
                # contacts (summary.csv's distinct_obstacles_detected) — never
                # written out per-object (see _sonar_columns/_SONAR_HEADER).
                "distinct_obstacles": set(),
                "contact_samples": 0,
                "battery_start_pct": None,
                "battery_end_pct": None,
                "sim_time_start": None,
                "sim_time_end": None,
                "final_arrival_error_m": "",
                "final_arrival_error_pct": "",
                "final_mission_complete": 0,
            }
            self._stats[agent] = stats
        return stats

    def _writer_for(self, agent: str):
        entry = self._files.get(agent)
        if entry is None:
            caps = self._capabilities(agent)
            header = list(_BASE_HEADER)
            if caps["battery"]:
                header += _BATTERY_HEADER
            if caps["mission"]:
                header += _MISSION_HEADER
            if caps["sonar"]:
                header += _SONAR_HEADER
            path = os.path.join(self._outdir, f"{self._prefix}{agent}.csv")
            f = open(path, "w", newline="")
            writer = csv.writer(f)
            writer.writerow(header)
            self._files[agent] = (f, writer)
            self.get_logger().info(f"New agent '{agent}' -> {path}")
            return writer
        return entry[1]

    def _mission_columns(self, agent: str, pos_xy: tuple) -> list:
        """Compute the mission_id/task_type/target/distance/cross-track/
        arrival columns for one agent at its current position. Returns
        blanks for agents with no known mission list (e.g. scenario wasn't
        passed in), and once all of an agent's missions are done.
        """
        specs = self._mission_specs.get(agent)
        # target_waypoint_idx, target_x, target_y, target_lat, target_lon,
        # distance_to_target_m, cross_track_error_m, along_track_progress_pct,
        # waypoint_arrived, arrival_error_m, arrival_error_pct — 11 columns,
        # matching the real tuple returned once a path is known (below).
        _BLANK_TRACKING = [""] * 11

        if not specs:
            return ["", "", *_BLANK_TRACKING, 0]

        if self._all_missions_complete.get(agent, False):
            last = specs[-1]
            return [last["id"], last["task"], *_BLANK_TRACKING, 1]

        midx = self._mission_idx[agent]
        mission = specs[midx]
        mission_id, task = mission["id"], mission["task"]

        if not mission["path"]:
            # No known target for this task type — report it as active but
            # can't track progress or auto-advance past it (see docstring).
            return [mission_id, task, *_BLANK_TRACKING, 0]

        path = mission["path"]
        leg_idx = self._leg_idx[agent]
        prev_xy = path[leg_idx - 1]
        target_xy = path[leg_idx]
        tolerance = mission["tolerance"]

        distance, cross_track, progress_pct = _leg_geometry(prev_xy, target_xy, pos_xy)
        target_lat, target_lon = _enu_to_latlon(
            target_xy[0], target_xy[1], self._ref_lat, self._ref_lon, self._ref_alt
        )

        stats = self._stats_for(agent)
        stats["max_cross_track_m"] = max(stats["max_cross_track_m"], cross_track)

        # Arrival is "did it EVER get within tolerance on this leg", not "is
        # it within tolerance RIGHT NOW" — see _leg_min_distance's docstring
        # in __init__. An agent that already arrived and stopped keeps
        # drifting under the current afterward, so the live `distance` alone
        # would make a genuinely-completed leg look unarrived again later.
        min_dist = min(self._leg_min_distance[agent], distance)
        self._leg_min_distance[agent] = min_dist
        arrived = min_dist <= tolerance
        arrival_error_m = ""
        arrival_error_pct = ""
        mission_complete = 0

        if arrived:
            leg_len = math.hypot(target_xy[0] - prev_xy[0], target_xy[1] - prev_xy[1])
            arrival_error_m = f"{min_dist:.3f}"
            arrival_error_pct = f"{(min_dist / leg_len * 100.0) if leg_len > 1e-6 else 0.0:.2f}"
            self.get_logger().info(
                f"'{agent}' arrived at waypoint {leg_idx} of mission '{mission_id}' "
                f"(error {min_dist:.2f} m, {arrival_error_pct}% of leg length)"
            )
            if leg_idx + 1 < len(path):
                self._leg_idx[agent] = leg_idx + 1
                self._leg_min_distance[agent] = float("inf")
            elif midx + 1 < len(specs):
                self._mission_idx[agent] = midx + 1
                self._leg_idx[agent] = 1
                self._leg_min_distance[agent] = float("inf")
                self.get_logger().info(
                    f"'{agent}' finished mission '{mission_id}', "
                    f"starting '{specs[midx + 1]['id']}' ({specs[midx + 1]['task']})"
                )
            else:
                self._all_missions_complete[agent] = True
                mission_complete = 1
                stats["final_arrival_error_m"] = arrival_error_m
                stats["final_arrival_error_pct"] = arrival_error_pct
                stats["final_mission_complete"] = 1
                self.get_logger().info(f"'{agent}' all missions complete.")

        return [
            mission_id, task,
            leg_idx,
            f"{target_xy[0]:.3f}", f"{target_xy[1]:.3f}",
            f"{target_lat:.8f}", f"{target_lon:.8f}",
            f"{distance:.3f}",
            f"{cross_track:.3f}",
            f"{progress_pct:.2f}",
            1 if arrived else 0,
            arrival_error_m,
            arrival_error_pct,
            mission_complete,
        ]

    def _active_sonar_range(self, agent: str) -> float:
        """The sonar_range_m of ``agent``'s currently active mission (0.0 if
        none/unknown — no sonar for this agent)."""
        specs = self._mission_specs.get(agent)
        if not specs or self._all_missions_complete.get(agent, False):
            return 0.0
        return specs[self._mission_idx[agent]].get("sonar_range_m", 0.0)

    def _sonar_columns(self, agent: str, pos_xy: tuple) -> list:
        """Fake sonar: distance to the nearest ``self._obstacle_agents``
        contact within this agent's active sonar_range_m, computed from
        ground-truth poses. Blank for agents with no sonar and for obstacle
        agents themselves. The nearest object's IDENTITY is used internally
        to pick the closest one and to count distinct contacts for
        summary.csv (``distinct_obstacles_detected``), but is never written
        to a column here — a real sonar reports range, not "this is mine0 vs
        mine1"; see ``_SONAR_HEADER``.
        """
        if agent in self._obstacle_agents:
            return ["", "", ""]
        sonar_range = self._active_sonar_range(agent)
        if sonar_range <= 0.0:
            return ["", "", ""]

        x, y = pos_xy
        nearest_name, nearest_dist = None, None
        for name in self._obstacle_agents:
            pose = self._poses.get(name)
            if pose is None:
                continue
            dist = math.hypot(pose.position.x - x, pose.position.y - y)
            if nearest_dist is None or dist < nearest_dist:
                nearest_name, nearest_dist = name, dist

        if nearest_name is None:
            return [f"{sonar_range:.1f}", 0, ""]

        stats = self._stats_for(agent)
        if stats["min_sonar_distance_m"] is None or nearest_dist < stats["min_sonar_distance_m"]:
            stats["min_sonar_distance_m"] = nearest_dist

        contact = 1 if nearest_dist <= sonar_range else 0
        if contact:
            stats["contact_samples"] += 1
            stats["distinct_obstacles"].add(nearest_name)
        # sonar_distance_m only reports a value while there's an actual
        # contact — a real sonar has no reading at all for something outside
        # its range, so leaking the ground-truth distance whenever an
        # obstacle merely exists (regardless of range) would contradict
        # sonar_contact and the "no more than a real sensor would know" rule
        # applied everywhere else here. min_sonar_distance_m (summary.csv)
        # still tracks the true closest approach unconditionally above —
        # that's a post-hoc difficulty indicator, not a live sensor reading.
        dist_str = f"{nearest_dist:.3f}" if contact else ""
        return [f"{sonar_range:.1f}", contact, dist_str]

    def _sample(self) -> None:
        if not self._poses:
            return
        with self._sim_time_lock:
            sim_time = self._sim_time
        if sim_time is None:
            sim_time = time.time() - self._t0

        for agent in sorted(self._poses):
            pose = self._poses[agent]
            caps = self._capabilities(agent)
            lat, lon = _enu_to_latlon(
                pose.position.x, pose.position.y,
                self._ref_lat, self._ref_lon, self._ref_alt,
            )
            row = [
                agent,
                f"{sim_time:.3f}",
                f"{pose.position.x:.3f}",
                f"{pose.position.y:.3f}",
                f"{pose.position.z:.3f}",
                f"{lat:.8f}",
                f"{lon:.8f}",
                f"{pose.orientation.x:.6f}",
                f"{pose.orientation.y:.6f}",
                f"{pose.orientation.z:.6f}",
                f"{pose.orientation.w:.6f}",
                self._current_vx,
                self._current_vy,
                self._current_vz,
            ]

            stats = self._stats_for(agent)
            if stats["sim_time_start"] is None:
                stats["sim_time_start"] = sim_time
            stats["sim_time_end"] = sim_time

            if caps["battery"]:
                battery = self._battery.get(agent)
                pct = (battery or {}).get("percentage", 0.0)
                # Only lock in battery_start_pct once a REAL reading has
                # arrived (battery topics are discovered async, ~1 s after
                # spawn — the first pose sample or two can predate that).
                # Locking in on the first sample regardless used to record
                # a bogus 0.0 "start" (the empty-dict fallback default, not
                # an actual reading), making every agent look like it
                # charged UP over the run instead of draining.
                if battery is not None and stats["battery_start_pct"] is None:
                    stats["battery_start_pct"] = pct
                if battery is not None:
                    stats["battery_end_pct"] = pct
                battery = battery or {}
                row += [
                    battery.get("voltage", 0.0),
                    battery.get("charge", 0.0),
                    battery.get("capacity", 0.0),
                    pct,
                    battery.get("status", "UNKNOWN"),
                ]
            if caps["mission"]:
                row += self._mission_columns(agent, (pose.position.x, pose.position.y))
            if caps["sonar"]:
                row += self._sonar_columns(agent, (pose.position.x, pose.position.y))

            self._writer_for(agent).writerow(row)
        for f, _w in self._files.values():
            f.flush()

        if (
            not self._finalized
            and self._trackable_agents
            and all(self._all_missions_complete.get(v, False) for v in self._trackable_agents)
        ):
            self._finalize(early=True)

    def _write_summary(self) -> None:
        """One row per agent ever sampled: objective, outcome, and
        difficulty indicators — a compact numeric fact sheet for this run
        (e.g. for an LLM to turn into a narrative report), not prose itself.
        """
        path = os.path.join(self._outdir, f"{self._prefix}summary.csv")
        header = [
            "agent_name", "sensors", "actuators",
            "start_lat", "start_lon", "final_target_lat", "final_target_lon",
            "mission_complete", "final_arrival_error_m", "final_arrival_error_pct",
            "max_cross_track_error_m",
            "min_sonar_distance_m", "distinct_obstacles_detected", "time_in_sonar_contact_s",
            "battery_start_pct", "battery_end_pct",
            "run_start_sim_time_s", "run_end_sim_time_s", "run_duration_s",
            "world_ref_lat", "world_ref_lon", "world_ref_alt",
            "ocean_current_vx_mps", "ocean_current_vy_mps", "ocean_current_vz_mps",
        ]
        try:
            with open(path, "w", newline="") as f:
                writer = csv.writer(f)
                writer.writerow(header)
                for agent in sorted(self._stats):
                    stats = self._stats[agent]
                    specs = self._mission_specs.get(agent)
                    caps = self._capabilities(agent)

                    start_lat = start_lon = final_lat = final_lon = ""
                    if specs:
                        first_path = next((m["path"] for m in specs if m["path"]), None)
                        last_path = next(
                            (m["path"] for m in reversed(specs) if m["path"]), None
                        )
                        if first_path:
                            start_lat, start_lon = _enu_to_latlon(
                                *first_path[0], self._ref_lat, self._ref_lon, self._ref_alt
                            )
                            start_lat, start_lon = f"{start_lat:.8f}", f"{start_lon:.8f}"
                        if last_path:
                            final_lat, final_lon = _enu_to_latlon(
                                *last_path[-1], self._ref_lat, self._ref_lon, self._ref_alt
                            )
                            final_lat, final_lon = f"{final_lat:.8f}", f"{final_lon:.8f}"

                    start_t, end_t = stats["sim_time_start"], stats["sim_time_end"]
                    duration = (end_t - start_t) if (start_t is not None and end_t is not None) else ""
                    contact_s = stats["contact_samples"] / self._rate_hz

                    writer.writerow([
                        agent,
                        ";".join(caps["sensors"]), ";".join(caps["actuators"]),
                        start_lat, start_lon, final_lat, final_lon,
                        stats["final_mission_complete"],
                        stats["final_arrival_error_m"], stats["final_arrival_error_pct"],
                        f"{stats['max_cross_track_m']:.3f}",
                        f"{stats['min_sonar_distance_m']:.3f}" if stats["min_sonar_distance_m"] is not None else "",
                        len(stats["distinct_obstacles"]),
                        f"{contact_s:.2f}",
                        stats["battery_start_pct"] if stats["battery_start_pct"] is not None else "",
                        stats["battery_end_pct"] if stats["battery_end_pct"] is not None else "",
                        f"{start_t:.3f}" if start_t is not None else "",
                        f"{end_t:.3f}" if end_t is not None else "",
                        f"{duration:.3f}" if duration != "" else "",
                        f"{self._ref_lat:.8f}", f"{self._ref_lon:.8f}", f"{self._ref_alt:.3f}",
                        self._current_vx, self._current_vy, self._current_vz,
                    ])
            self.get_logger().info(f"Wrote run summary -> {path}")
        except Exception:
            self.get_logger().exception("Could not write summary.csv")

    def _finalize(self, early: bool = False) -> None:
        """Stop recording and write the summary. Idempotent — safe to call
        both from ``_sample()`` (all tracked agents arrived, simulation
        keeps running) and from ``destroy_node()`` (full process shutdown,
        possibly with some missions still incomplete).
        """
        if self._finalized:
            return
        self._finalized = True
        if self._sample_timer is not None:
            self._sample_timer.cancel()
        if self._battery_timer is not None:
            self._battery_timer.cancel()
        if self._gz_node is not None:
            # gz-transport's Node runs its subscription callback on its own
            # native (non-rclpy) thread. Left to Python's garbage collector,
            # it can be torn down at an arbitrary point during interpreter
            # shutdown — including while that thread is mid-callback into
            # Python — which segfaults. Unsubscribing explicitly here, before
            # anything else starts tearing down, makes the teardown ordering
            # deterministic instead of GC-timing-dependent.
            try:
                self._gz_node.unsubscribe("/stats")
            except Exception:
                pass
        self._write_summary()
        for f, _w in self._files.values():
            try:
                f.close()
            except Exception:
                pass
        if early:
            self.get_logger().info(
                "All tracked missions complete: CSV recording stopped and "
                "summary written. The simulation itself keeps running."
            )

    def destroy_node(self) -> None:
        # Catch up before writing the summary: this recorder's own arrival/
        # mission_complete detection runs on its 2 Hz sample loop, separate
        # from a vehicle's actual 20 Hz guidance-loop arrival check (which is
        # what logs the real "Mission '<id>' finished with SUCCESS"). If
        # shutdown lands between two scheduled samples, the recorder can
        # still show mission_complete=0 for a vehicle that has genuinely
        # already arrived — one last sample here (using whatever pose is
        # currently on /poses, which already reflects the true final
        # position by the time shutdown happens) closes that gap. Skipped if
        # recording already stopped itself early (§_finalize).
        if not self._finalized:
            try:
                self._sample()
            except Exception:
                self.get_logger().exception("Final catch-up sample before shutdown failed")
        self._finalize()
        super().destroy_node()


def recorder_from_config(record_cfg, world_name: str, scenario: dict = None):
    """Build a :class:`CsvRecorder` from the scenario JSON ``record_csv`` value.

    ``record_cfg`` may be ``True`` (all defaults) or a dict with optional
    ``rate`` (Hz), ``outdir``, ``prefix``, ``ref_lat``, ``ref_lon`` and
    ``ref_alt`` keys. Returns None when disabled.

    ``scenario`` should be the *whole* parsed scenario JSON dict (the one
    with the top-level ``"agents"`` list), not just the ``record_csv`` value.
    When provided, each agent's ``spawn`` + ordered ``missions`` list is used
    to reconstruct the intended path per agent and to report which mission
    is currently active, enabling the mission/target/distance/cross-track/
    arrival columns described in the module docstring. Pass ``None`` (or
    omit it) to skip mission tracking and only get the pose/battery/lat-lon
    columns.

    ``ref_lat``/``ref_lon``/``ref_alt`` should match the world SDF's
    ``<spherical_coordinates>`` block so the recorded lat/lon (and the
    waypoint conversion, if ``scenario`` is given) line up correctly. They
    default to energy.world's origin.

    Default output directory: ``$LOG_DIR/csv`` when scenario_launch.sh exported
    LOG_DIR (the CSVs then land next to that run's logs), else
    ``./csv_logs_<world>_<timestamp>``.
    """
    if not record_cfg:
        return None
    cfg = record_cfg if isinstance(record_cfg, dict) else {}
    log_dir = os.environ.get("LOG_DIR")
    default_outdir = (
        os.path.join(log_dir, "csv")
        if log_dir
        else os.path.join(
            os.getcwd(), f"csv_logs_{world_name}_{time.strftime('%Y-%m-%d_%H-%M-%S')}"
        )
    )
    ref_lat = float(cfg.get("ref_lat", _DEFAULT_REF_LAT))
    ref_lon = float(cfg.get("ref_lon", _DEFAULT_REF_LON))
    ref_alt = float(cfg.get("ref_alt", _DEFAULT_REF_ALT))

    mission_specs = (
        _mission_specs_from_scenario(scenario, ref_lat, ref_lon, ref_alt)
        if scenario else {}
    )
    obstacle_agents = _obstacle_agents_from_scenario(scenario) if scenario else set()
    agent_capabilities = (
        _agent_capabilities_from_scenario(scenario) if scenario else {}
    )
    ocean_current = _ocean_current_from_scenario(scenario or {})

    return CsvRecorder(
        world=world_name,
        outdir=cfg.get("outdir", default_outdir),
        prefix=cfg.get("prefix", ""),
        rate_hz=float(cfg.get("rate", 2.0)),
        ref_lat=ref_lat,
        ref_lon=ref_lon,
        ref_alt=ref_alt,
        mission_specs=mission_specs,
        obstacle_agents=obstacle_agents,
        agent_capabilities=agent_capabilities,
        ocean_current=ocean_current,
    )