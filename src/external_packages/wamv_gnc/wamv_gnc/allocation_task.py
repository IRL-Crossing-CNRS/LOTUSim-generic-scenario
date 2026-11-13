"""Allocation block: wrench -> the two WAMV stern thruster commands.

Subscribes ``geometry_msgs/WrenchStamped`` on ``/<world>/<agent>/control``;
publishes ``lotusim_msgs/VesselCmdArray`` on ``/<world>/vessel_cmd_array``,
same slot BlueROV's allocator uses. WAMV's xdyn model
(``assets/models/wamv/wamv.yaml``) has two `maneuvering` thrusters, one per
hull, each commanded by a thrust ``T`` in newtons along the body x axis:

    prop    position (x, y, z) m              force per N of T
    left    (-2.374, -1.027, 0.191)           X = +1
    right   (-2.374, +1.027, 0.191)           X = +1

(VRX's engine positions), so ``X = T_left + T_right`` and
``N = 1.027 * (T_left - T_right)`` (NED: a positive N turns the bow to
starboard). Inverting:

    T_left  = X / 2 + N / (2 * 1.027)
    T_right = X / 2 - N / (2 * 1.027)

Sway and heave are not actuated: the demand on those axes is dropped.
On saturation the yaw share is kept and the surge share is scaled down, so
the vehicle keeps steering when it is asked for more thrust than it has.
"""

from __future__ import annotations

import json

from geometry_msgs.msg import WrenchStamped
from lotusim_msgs.msg import VesselCmd, VesselCmdArray

from lotusim_sdk.bt.status import Status
from lotusim_sdk.tasks.base import TaskAgent

#: Half the lateral spacing of the two thrusters (m), from wamv.yaml.
HALF_SPACING_M = 1.027

#: Names of the `maneuvering` force models in wamv.yaml.
PROPS = ("wamv_prop_left", "wamv_prop_right")


class WamvAllocationTask(TaskAgent):
    """Maps a (surge, yaw) wrench demand to the two thruster commands.

    Params:
        t_max_n  float   per-thruster saturation, forward and reverse (default 250)
    """

    PERPETUAL = True

    def __init__(self, host, params=None, blackboard=None, id: str = "") -> None:
        super().__init__(host, params, blackboard, id)
        self._t_max = float(self.params.get("t_max_n", 250.0))
        self._sub = None
        self._pub = None

    def on_enter(self) -> None:
        world = self.host.world_name
        agent = self.host.agent_name
        self._pub = self.host.create_publisher(VesselCmdArray, f"/{world}/vessel_cmd_array", 10)
        self._publish(0.0, 0.0)
        self._sub = self.host.create_subscription(
            WrenchStamped, f"/{world}/{agent}/control", self._on_control, 10
        )

    def on_exit(self, status) -> None:
        if self._sub is not None:
            self.host.destroy_subscription(self._sub)
            self._sub = None
        self._publish(0.0, 0.0)

    def update(self) -> Status:
        return Status.RUNNING

    def to_commands(self, surge_n: float, yaw_nm: float) -> tuple:
        """surge > 0 forward, yaw > 0 to starboard -> (T_left, T_right)."""
        diff = max(-self._t_max, min(self._t_max, yaw_nm / (2.0 * HALF_SPACING_M)))
        room = self._t_max - abs(diff)
        common = max(-room, min(room, surge_n / 2.0))
        return common + diff, common - diff

    def _on_control(self, msg: WrenchStamped) -> None:
        self._publish(*self.to_commands(msg.wrench.force.x, msg.wrench.torque.z))

    def _publish(self, t_left: float, t_right: float) -> None:
        cmd = VesselCmd()
        cmd.vessel_name = self.host.agent_name
        cmd.cmd_string = json.dumps({f"{PROPS[0]}(T)": float(t_left), f"{PROPS[1]}(T)": float(t_right)})
        array = VesselCmdArray()
        array.cmds = [cmd]
        self._pub.publish(array)
