"""Static allocation block: publishes one fixed, never-varying xdyn command.

For an xdyn vehicle whose force models *declare* actuator command keys
(e.g. a propeller), xdyn stalls every step until it receives a complete
command set (see ``xdyn_websocket.cpp``). That's true even
if the desired behaviour is simply "let it drift", the way a hull-only
vehicle (no declared actuators at all) already does for free. This task
is the actuator-bearing equivalent: it publishes a fixed resting command
(e.g. a propeller at zero rpm) once on enter and repeats it on every tick,
but never reads Control's output and never varies it -- there is no
feedback loop, so there is no gain to tune and nothing that can diverge.
The vehicle still drifts under real hydrodynamics + current, with its
actuator(s) held at rest, same as a hull-only vessel modulo whatever
parasitic drag the resting actuator itself contributes.

This is not a substitute for a real Allocation task -- it never responds
to Control's demand, so the vehicle is not controllable while this task
runs. It exists for exactly one purpose: a "drift-only" demo for a vehicle
that has actuators declared in its xdyn YAML (so the hull-only "just omit
Allocation" trick doesn't apply), without touching whatever is or isn't
resolved about that vehicle's real Allocation gains/hydrodynamic model.

Params:
    commands  dict   passed straight through as the cmd_string JSON, e.g.
                      {"propeller(rpm)": 0.0, "propeller(P/D)": 0.88} for
                      LRAUV, or the twin-propeller+rudder equivalent for
                      dtmb_hull. No default -- a scenario must supply the
                      actuator's own command keys and a sensible resting
                      value for each (usually 0 for rpm/angle, the
                      vehicle's own documented default for a fixed ratio
                      like P/D).
"""

from __future__ import annotations

import json

from lotusim_msgs.msg import VesselCmd, VesselCmdArray

from lotusim_sdk.bt.status import Status
from lotusim_sdk.tasks.base import TaskAgent


class StaticCommandAllocationTask(TaskAgent):
    """Publishes a fixed ``commands`` dict as the vehicle's xdyn command,
    unconditionally and forever -- see module docstring.
    """

    PERPETUAL = True

    def __init__(self, host, params=None, blackboard=None, id: str = "") -> None:
        super().__init__(host, params, blackboard, id)
        self._commands = dict(self.params.get("commands", {}))
        self._pub = None

    def on_enter(self) -> None:
        world = self.host.world_name
        self._pub = self.host.create_publisher(VesselCmdArray, f"/{world}/vessel_cmd_array", 10)
        self._publish()

    def on_exit(self, status) -> None:
        pass

    def update(self) -> Status:
        # Republished every tick, not just once on enter: matches a real
        # Allocation task's behaviour (it republishes on every Control
        # message) rather than relying on DDS to redeliver a single old
        # message to whatever's on the other end of the websocket bridge.
        self._publish()
        return Status.RUNNING

    def _publish(self) -> None:
        cmd = VesselCmd()
        cmd.vessel_name = self.host.agent_name
        cmd.cmd_string = json.dumps(self._commands)
        array = VesselCmdArray()
        array.cmds = [cmd]
        self._pub.publish(array)
