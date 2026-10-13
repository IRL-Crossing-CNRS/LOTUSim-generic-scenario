"""Allocation block: wrench -> per-thruster commands.

Subscribes ``geometry_msgs/WrenchStamped`` on ``/<world>/<agent>/control``;
publishes ``lotusim_msgs/VesselCmdArray`` (the per-thruster command message
the simulator's ``XdynWebsocket`` consumes) on ``/<world>/vessel_cmd_array``.
``ThrusterAllocator`` inverts this vehicle's own thruster layout (see
``thruster_allocation.py``); a different vehicle needs a different allocator,
wired the same way.
"""

from __future__ import annotations

import json

from geometry_msgs.msg import WrenchStamped
from lotusim_msgs.msg import VesselCmd, VesselCmdArray

from lotusim_sdk.bt.status import Status
from lotusim_sdk.tasks.base import TaskAgent

from .thruster_allocation import ThrusterAllocator

#: The six modelled thrusters. A BlueROV2 *Heavy* has eight: 6 and 7 are absent
#: from this vehicle model, so roll and pitch are not controllable.
PROPS = (1, 2, 3, 4, 5, 8)

#: Prefix of the `maneuvering` force models in the xdyn YAML.
BODY = "bluerov2_heavy"


class BlueRovAllocationTask(TaskAgent):
    """Maps a wrench demand to the six BlueROV2 Heavy thruster commands.

    Params:
        t_max_n  float   per-thruster saturation (default 50, T200)
    """

    PERPETUAL = True

    def __init__(self, host, params=None, blackboard=None, id: str = "") -> None:
        super().__init__(host, params, blackboard, id)
        self._alloc = ThrusterAllocator(t_max=float(self.params.get("t_max_n", 50.0)))
        self._sub = None
        self._pub = None

    def on_enter(self) -> None:
        world = self.host.world_name
        agent = self.host.agent_name
        self._pub = self.host.create_publisher(VesselCmdArray, f"/{world}/vessel_cmd_array", 10)
        # The plugin seeds the command map either from the SDF <thrusters> tag
        # or from <initial_commands>; this package's agent declares
        # THRUSTERS = [], so publish a zero command right away to guarantee
        # the very first xdyn step has all six keys.
        self._publish({i: 0.0 for i in PROPS})
        self._sub = self.host.create_subscription(WrenchStamped, f"/{world}/{agent}/control", self._on_control, 10)

    def on_exit(self, status) -> None:
        if self._sub is not None:
            self.host.destroy_subscription(self._sub)
            self._sub = None
        self._publish({i: 0.0 for i in PROPS})

    def update(self) -> Status:
        return Status.RUNNING

    def _on_control(self, msg: WrenchStamped) -> None:
        thrusts = self._alloc.to_commands(
            msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z, msg.wrench.torque.z
        )
        self._publish(thrusts)

    def _publish(self, thrusts) -> None:
        cmd = VesselCmd()
        cmd.vessel_name = self.host.agent_name
        cmd.cmd_string = json.dumps({f"{BODY}_prop_{i}(T)": float(thrusts[i]) for i in PROPS})
        array = VesselCmdArray()
        array.cmds = [cmd]
        self._pub.publish(array)
