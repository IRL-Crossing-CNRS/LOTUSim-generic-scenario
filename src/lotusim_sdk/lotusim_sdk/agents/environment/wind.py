from dataclasses import dataclass
from typing import List, Optional

from rclpy.qos import DurabilityPolicy, QoSProfile

from geometry_msgs.msg import Vector3
from lotusim_msgs.msg import Wind as WindMsg
from lotusim_msgs.msg import WindRegion as WindRegionMsg
from lotusim_msgs.msg import WindRegionArray as WindRegionArrayMsg

from lotusim_sdk.agents.environment import Environment
from lotusim_sdk.bt.status import Status

# The one *ambient* wind topic in the system. Hardcoded, like everything else
# about aerialWorld: the Unity wind sliders publish here, the wind_regions
# Gazebo plugin subscribes to it directly (no bridge process involved), and
# the Wake agent reads it. This agent is simply another writer on that same
# topic — a slider driven from the scenario JSON instead of by hand, and only
# while a mission actually claims it.
WIND_TOPIC = "/aerialWorld/wind"

# Per-region wind overrides, applied inside each region's 2D box (all
# altitudes) on top of the ambient vector above. Always published — unlike
# WIND_TOPIC this is not gated by mission activity, since a `mirror_global`
# region must keep tracking the ambient vector no matter who is driving it.
WIND_REGIONS_TOPIC = "/aerialWorld/wind/regions"

_TERMINAL = (Status.SUCCESS, Status.FAILURE)


@dataclass
class WindRegion:
    """A 2D box (x1,y1)-(x2,y2), all altitudes, with its own wind vector.

    If ``mirror_global`` is set, ``x``/``y``/``z`` are ignored and the
    published vector is always ``-1 * (current ambient wind)`` instead — used
    to exercise the region pipeline with a vector that changes dynamically
    (from the Unity sliders or a scripted ``Wind`` mission) without needing a
    dedicated BT task to drive it.
    """

    id: str
    x1: float
    y1: float
    x2: float
    y2: float
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    mirror_global: bool = False


class Wind(Environment):
    """Ambient wind vector, driven by a mission instead of by the Unity sliders —
    but only for as long as that mission is actually running. Optionally also
    holds a static list of 2D wind regions overriding the ambient vector
    inside their box.

    Holds one ENU vector (x=East, y=North, z=Up, m/s) and, while active,
    publishes it on :data:`WIND_TOPIC` — the exact topic and message type the
    Unity wind sliders write to. ``set_wind`` tasks in this agent's ``missions``
    change it.

    **Passive by default, active only mid-mission.** With no ``missions``
    declared (or once every declared mission has reached SUCCESS/FAILURE) this
    agent publishes nothing at all on :data:`WIND_TOPIC` — the sliders are the
    only writer on the topic and free hand-tuning works exactly as if no
    ``Wind`` agent existed. The moment a mission is set (even before its first
    tick — see :meth:`_is_active`) it starts steadily overwriting the topic
    with its own held vector, precisely because a scripted wind history would
    otherwise be stomped on by whatever the sliders happen to be at; once the
    mission tree finishes, this agent publishes its held vector exactly one
    more time — so a closing ``set_wind`` (e.g. resetting to 0 as a mission's
    last step, which runs in the same tick that reaches SUCCESS) actually
    reaches the topic instead of being silently dropped — and then stops for
    good, handing the topic back to the sliders.

    Declaring a ``Wind`` agent with no ``missions`` is a legitimate no-op: it
    exists purely so a scenario can later add wind missions without introducing
    a new agent.

    Regions, by contrast, are always published (see :data:`WIND_REGIONS_TOPIC`)
    regardless of mission state — this agent also subscribes to its own
    :data:`WIND_TOPIC` so it always knows the currently effective ambient
    vector, whether it came from its own mission or from the Unity sliders,
    and any ``mirror_global`` region reflects that value live.
    """

    def __init__(
        self,
        world_name: str,
        x: float = 0.0,
        y: float = 0.0,
        z: float = 0.0,
        publish_rate_hz: float = 10.0,
        regions: Optional[List[dict]] = None,
        **kwargs,
    ):
        super().__init__(world_name)

        self._vector = (float(x), float(y), float(z))
        # Latest ambient vector in effect, whoever wrote it — seeded with our
        # own initial vector, then kept current by _on_ambient_wind below.
        self._last_known_ambient = self._vector
        # Tracks _is_active() across publish ticks — see _publish().
        self._was_active = False

        self._regions: List[WindRegion] = [
            WindRegion(
                id=str(region.get("id", f"region_{i}")),
                x1=float(region["x1"]),
                y1=float(region["y1"]),
                x2=float(region["x2"]),
                y2=float(region["y2"]),
                x=float(region.get("x", 0.0)),
                y=float(region.get("y", 0.0)),
                z=float(region.get("z", 0.0)),
                mirror_global=bool(region.get("mirror_global", False)),
            )
            for i, region in enumerate(regions or [])
        ]

        # Latched so the wind_regions Gazebo plugin, which comes up
        # independently, does not sit at zero/no-region wind until the next
        # publish tick.
        latched_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._pub = self.create_publisher(WindMsg, WIND_TOPIC, latched_qos)
        self._regions_pub = self.create_publisher(WindRegionArrayMsg, WIND_REGIONS_TOPIC, latched_qos)

        # Plain (volatile) QoS: WIND_TOPIC may also be written directly by the
        # Unity sliders over ROS-TCP, whose publisher durability is not
        # controlled here. Requesting TRANSIENT_LOCAL here would risk a
        # silent QoS incompatibility (zero delivery) against a volatile
        # publisher — volatile accepts either durability, so it's the only
        # safe choice for a subscription to a topic not fully owned here.
        self._ambient_sub = self.create_subscription(WindMsg, WIND_TOPIC, self._on_ambient_wind, 10)

        self.create_timer(1.0 / publish_rate_hz, self._publish)

        self.get_logger().info(
            f"Wind ready (initial [{x}, {y}, {z}] m/s, {len(self._regions)} region(s)) — "
            f"passive on {WIND_TOPIC} until a mission runs; regions on {WIND_REGIONS_TOPIC} "
            f"always publish."
        )

    def set_wind(self, x: float = None, y: float = None, z: float = None) -> None:
        """Set the wind vector in m/s (ENU). Omitted components keep their value.

        The whole tuple is swapped in one bound assignment, so no lock is needed:
        the publish timer reads whichever tuple is current and can never observe
        a half-updated vector.
        """
        cur_x, cur_y, cur_z = self._vector
        self._vector = (
            cur_x if x is None else float(x),
            cur_y if y is None else float(y),
            cur_z if z is None else float(z),
        )

    def get_wind(self) -> tuple:
        """Return the current wind vector as ``(x, y, z)`` in m/s (ENU)."""
        return self._vector

    def _is_active(self) -> bool:
        """True while at least one mission root hasn't reached a terminal status.

        A root defaults to ``Status.RUNNING`` before its first tick (see
        ``BehaviorNode.__init__``), so this is already true the instant
        ``set_missions`` installs a wind mission — the topic is claimed before
        the mission engine's own timer gets a chance to tick it, not after.
        """
        return any(root.status not in _TERMINAL for root in self._missions)

    def _on_ambient_wind(self, msg: WindMsg) -> None:
        """Track the ambient vector currently in effect, regardless of writer
        (this agent mid-mission, or the Unity sliders while passive), so
        ``mirror_global`` regions always reflect the live value."""
        self._last_known_ambient = (
            msg.linear_velocity.x,
            msg.linear_velocity.y,
            msg.linear_velocity.z,
        )

    def _publish(self) -> None:
        # Publish while active, AND for the one tick where a mission just
        # went terminal — otherwise a closing ``set_wind`` (e.g. resetting to
        # 0 as the last step of a sequence) runs in the same BT tick that
        # flips the mission to SUCCESS/FAILURE, so ``_is_active()`` is
        # already False by the time the next publish timer tick checks it:
        # that final value would never actually reach the (latched) topic,
        # leaving whichever vector was held during the previous step stuck
        # there instead of handing back a clean value to the sliders.
        active = self._is_active()
        if active or self._was_active:
            x, y, z = self._vector
            msg = WindMsg()
            msg.linear_velocity = Vector3(x=x, y=y, z=z)
            msg.enable_wind = True
            self._pub.publish(msg)
        self._was_active = active
        self._publish_regions()

    def _publish_regions(self) -> None:
        if not self._regions:
            return
        gx, gy, gz = self._last_known_ambient
        msg = WindRegionArrayMsg()
        for region in self._regions:
            region_msg = WindRegionMsg()
            region_msg.id = region.id
            region_msg.shape_type = WindRegionMsg.BOX
            region_msg.box.x1, region_msg.box.y1 = region.x1, region.y1
            region_msg.box.x2, region_msg.box.y2 = region.x2, region.y2
            if region.mirror_global:
                region_msg.linear_velocity = Vector3(x=-gx, y=-gy, z=-gz)
            else:
                region_msg.linear_velocity = Vector3(x=region.x, y=region.y, z=region.z)
            region_msg.enable_wind = True
            msg.regions.append(region_msg)
        self._regions_pub.publish(msg)

    def destroy_node(self) -> None:
        """Clear the region list on shutdown: Unity treats an empty
        ``WindRegionArray`` as "delete every box", so publishing one here
        before the node goes away wipes this run's region visuals instead of
        leaving them stuck on screen (the topic is latched, so without this
        the last non-empty message would otherwise persist for whoever
        starts the next scenario)."""
        self._regions_pub.publish(WindRegionArrayMsg())
        super().destroy_node()
