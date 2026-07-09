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
_shared_pose_subscriptions: dict[str, object] = {}
_shared_pose_registry_lock = threading.Lock()


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

    @property
    def current_pose(self):
        return _shared_pose_tables.get(self.world_name, {}).get(self.agent_name)

    @property
    def last_pose_update(self) -> float:
        return _shared_pose_stamps.get(self.world_name, 0.0)

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
            _rename_entity(self.world_name, self.agent_name, assigned_name, self)
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

        def _on_goal(gf):
            try:
                goal_handle = gf.result()
            except Exception:
                return
            if goal_handle is None or not goal_handle.accepted:
                self.get_logger().error(f"{self.agent_name}: spawn REJECTED by host.")
                return

            def _on_result(rf):
                try:
                    assigned = rf.result().result.name
                except Exception:
                    return
                if not assigned or assigned == "error_cmd":
                    self.get_logger().error(f"{self.agent_name}: host failed to spawn entity.")
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

    def send_single_mas_cmd_geo(self, lat, lon, alt=0.0, server_timeout_sec: float = 5.0):
        goal_msg = lotusim_msgs.action.MASCmd.Goal()
        cmd = MASCmd()
        cmd.cmd_type = MASCmd.CREATE_CMD
        cmd.model_name = self.model_name
        cmd.sdf_file = self.sdf_file
        cmd.vessel_name = self.agent_name
        cmd.sdf_string = self.lotus_param()

        geo = GeoPoint()
        geo.latitude = float(lat)
        geo.longitude = float(lon)
        geo.altitude = float(alt)
        cmd.geo_point = geo
        goal_msg.cmd = cmd

        self.get_logger().info(f"Sending MAS command with GeoPoint: lat={lat}, lon={lon}, alt={alt}")

        if not self.mas_action_client.wait_for_server(timeout_sec=server_timeout_sec):
            self.get_logger().error(f"{self.agent_name}: MASCmd server unavailable.")
            return None
        goal_future = self.mas_action_client.send_goal_async(goal_msg)
        self._attach_spawn_result_handler(goal_future)
        return goal_future

    def send_single_mas_cmd_pose(self, pose, server_timeout_sec: float = 5.0):
        goal_msg = lotusim_msgs.action.MASCmd.Goal()
        cmd = MASCmd()
        cmd.cmd_type = MASCmd.CREATE_CMD
        cmd.model_name = self.model_name
        cmd.sdf_file = self.sdf_file
        cmd.vessel_name = self.agent_name
        cmd.sdf_string = self.lotus_param()

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
        cmd.vessel_position = pose_msg
        goal_msg.cmd = cmd

        self.get_logger().info(f"Sending MAS command with XYZ pose: {pose}")

        if not self.mas_action_client.wait_for_server(timeout_sec=server_timeout_sec):
            self.get_logger().error(f"{self.agent_name}: MASCmd server unavailable.")
            return None
        goal_future = self.mas_action_client.send_goal_async(goal_msg)
        self._attach_spawn_result_handler(goal_future)
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
