"""
@file ocean_current.py
@brief Publishes a fake, uniform ocean current to the host physics plugin.

Enable it from the scenario JSON (top-level, sibling of ``"agents"``)::

    { "ocean_current": {"vx": 0.05, "vy": 0.12}, ... }

``vx``/``vy`` are a world-frame ENU velocity in m/s (x=East, y=North), added
by the host's ``KinematicInterface`` (``physics_engine_interface`` Gazebo
plugin) to every Kinematic-connected entity's own commanded velocity —
vehicles running a ``waypoint_follower``/``waypoint_follower_avoidance``
mission, and any prop using the ``kinematic_anchor`` task (e.g. mines). This
is deliberately NOT xdyn: xdyn agents get a physically-simulated current
from their own hydrodynamic config instead (see ``mineConfig.yaml`` and
friends) — this publisher only feeds the Kinematic path, which is what the
Raj scenario (and any xdyn-free scenario) actually uses.

The message is published once, with TRANSIENT_LOCAL durability, so it is
delivered to the Gazebo plugin whenever its subscription comes up, no matter
the relative startup order between gzserver and this process. The publishing
node must stay alive/registered on the executor for the run's duration for
that late-joiner delivery to work — see how ``recorder_from_config`` is used
in ``simulation_runner.run_simulation`` for the same pattern.
"""

from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import Vector3


class OceanCurrentPublisher(Node):
    """Latches a single world-frame ENU current vector for the host plugin."""

    def __init__(self, world: str, vx: float, vy: float) -> None:
        super().__init__("ocean_current_publisher")
        pub = self.create_publisher(
            Vector3,
            f"/{world}/ocean_current",
            QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )
        msg = Vector3()
        msg.x = float(vx)
        msg.y = float(vy)
        pub.publish(msg)
        self.get_logger().info(
            f"Ocean current set to ({vx}, {vy}) m/s (ENU) on /{world}/ocean_current"
        )


def current_from_config(current_cfg, world_name: str):
    """Build an :class:`OceanCurrentPublisher` from the scenario JSON
    ``ocean_current`` value. Returns None when disabled/absent."""
    if not current_cfg:
        return None
    cfg = current_cfg if isinstance(current_cfg, dict) else {}
    return OceanCurrentPublisher(
        world=world_name,
        vx=float(cfg.get("vx", 0.0)),
        vy=float(cfg.get("vy", 0.0)),
    )
