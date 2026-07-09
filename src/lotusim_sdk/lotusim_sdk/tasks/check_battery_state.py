from __future__ import annotations

from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool

from lotusim_sdk.bt.status import Status
from lotusim_sdk.tasks.base import TaskAgent


class CheckBatteryStateTask(TaskAgent):
    """Drive the agent's status LED from its battery level.

    Reads the battery state the ``battery_sensor`` publishes natively on ROS
    (``/<world>/<agent>/battery/state``) and commands the ``light_actuator``
    (in the model SDF) over ``/<world>/<agent>/light/cmd``:

        battery percentage > threshold  →  light ON
        otherwise                        →  light OFF

    Event-driven, like ``FaultInspectionTask``: ``update`` always returns
    RUNNING; the real work happens in the battery callback, which only publishes
    when the desired light state changes (edge-triggered).

    Params (mission JSON ``"params"``):
        threshold  float  battery %% above which the light is ON (default 80.0).
                          The sensor reports percentage on a 0-100 scale here
                          (``fix_issue_225`` is set in the model).
    """

    def __init__(self, host, params=None, blackboard=None, id: str = "") -> None:
        super().__init__(host, params, blackboard, id)
        self._threshold: float = float(self.params.get("threshold", 80.0))
        self._battery_sub = None
        self._light_pub = None
        self._last_cmd: bool | None = None

    # -- lifecycle ------------------------------------------------------------

    def on_enter(self) -> None:
        world = self.host.world_name
        agent = self.host.agent_name

        # Match the battery_sensor publisher QoS (TRANSIENT_LOCAL) so we still get
        # the latest reading even if it published before this leaf became active.
        self._battery_sub = self.host.create_subscription(
            BatteryState,
            f"/{world}/{agent}/battery/state",
            self._battery_callback,
            QoSProfile(
                depth=10,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )
        # TRANSIENT_LOCAL so the light_actuator gets the last command even if it
        # subscribes slightly after we first publish.
        self._light_pub = self.host.create_publisher(
            Bool,
            f"/{world}/{agent}/light/cmd",
            QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )
        self._last_cmd = None
        self.host.get_logger().info(
            f"CheckBatteryStateTask: {agent} LED follows battery "
            f"(> {self._threshold}% = ON)."
        )

    def update(self) -> Status:
        return Status.RUNNING

    def on_exit(self, _status: Status) -> None:
        if self._battery_sub is not None:
            self.host.destroy_subscription(self._battery_sub)
            self._battery_sub = None
        if self._light_pub is not None:
            self.host.destroy_publisher(self._light_pub)
            self._light_pub = None

    # -- algo -----------------------------------------------------------------

    def _battery_callback(self, msg: BatteryState) -> None:
        if self._light_pub is None:
            return
        desired_on = msg.percentage > self._threshold
        if desired_on == self._last_cmd:
            return  # edge-triggered: only publish on change
        self._last_cmd = desired_on
        self._light_pub.publish(Bool(data=desired_on))
        self.host.get_logger().info(
            f"CheckBatteryStateTask: battery {msg.percentage:.1f}% "
            f"→ LED {'ON' if desired_on else 'OFF'}"
        )
