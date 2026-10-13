from rclpy.qos import DurabilityPolicy, QoSProfile

from geometry_msgs.msg import Vector3
from lotusim_msgs.msg import OceanCurrent as OceanCurrentMsg

from lotusim_sdk.agents.environment import Environment

OCEAN_CURRENT_TOPIC_FMT = "/{world}/ocean_current"


class OceanCurrent(Environment):
    """Ambient ocean current vector (ENU, m/s), fed to the host's
    KinematicInterface Gazebo plugin (``physics_engine_interface``, x/y only —
    its pose integration is 2D) and to whatever visualizes current vectors
    (e.g. Unity).

    Holds one ENU vector (x=East, y=North, z=Up, m/s) and continuously
    republishes it, latched, on ``/{world}/ocean_current`` so a late
    subscriber (the Gazebo plugin, Unity) always gets the current value
    regardless of startup order. ``set_current`` tasks in this agent's
    ``missions`` change it.

    On shutdown, publishes a disabled (``enable_current=False``) message —
    the topic is latched, so without this the last non-empty value would
    otherwise persist for whoever subscribes next (the next scenario's
    plugin instance, or Unity), instead of being told the current is gone.
    """

    def __init__(
        self,
        world_name: str,
        x: float = 0.0,
        y: float = 0.0,
        z: float = 0.0,
        publish_rate_hz: float = 1.0,
        **kwargs,
    ):
        super().__init__(world_name)

        self._vector = (float(x), float(y), float(z))

        # Latched so the KinematicInterface plugin and Unity, which come up
        # independently, do not sit at zero current until the next publish
        # tick.
        latched_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._topic = OCEAN_CURRENT_TOPIC_FMT.format(world=world_name)
        self._pub = self.create_publisher(OceanCurrentMsg, self._topic, latched_qos)

        self.create_timer(1.0 / publish_rate_hz, self._publish)
        self._publish()

        self.get_logger().info(f"OceanCurrent ready (initial [{x}, {y}, {z}] m/s) on {self._topic}")

    def set_current(self, x: float = None, y: float = None, z: float = None) -> None:
        """Set the ocean current vector in m/s (ENU). Omitted components keep their value."""
        cur_x, cur_y, cur_z = self._vector
        self._vector = (
            cur_x if x is None else float(x),
            cur_y if y is None else float(y),
            cur_z if z is None else float(z),
        )

    def get_current(self) -> tuple:
        """Return the current ocean current vector as ``(x, y, z)`` in m/s (ENU)."""
        return self._vector

    def _publish(self) -> None:
        x, y, z = self._vector
        msg = OceanCurrentMsg()
        msg.linear_velocity = Vector3(x=x, y=y, z=z)
        msg.enable_current = True
        self._pub.publish(msg)

    def destroy_node(self) -> None:
        """Publish a disabled current message on shutdown, see class docstring."""
        self._pub.publish(OceanCurrentMsg())
        super().destroy_node()
