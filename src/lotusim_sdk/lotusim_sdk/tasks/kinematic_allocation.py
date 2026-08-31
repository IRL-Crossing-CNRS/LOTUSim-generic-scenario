"""Kinematic allocation block: wrench -> {u, w, vz} for KinematicInterface.

Subscribes ``geometry_msgs/WrenchStamped`` on ``/<world>/<agent>/control``;
publishes ``lotusim_msgs/VesselCmdArray`` with a
``{"u": ..., "w": ..., "vz": ...}`` ``cmd_string`` on
``/<world>/vessel_cmd_array`` -- the format
``KinematicInterface::getNewState()`` (in the host's physics interface
plugin) already parses. This is the Kinematic-side counterpart of
``allocation_task.py``'s ThrusterAllocator: same input (a wrench from
Control), same output slot (``vessel_cmd_array``), so a Kinematic-backed
vehicle can sit downstream of the exact same generic Control stage as an
xdyn-backed one -- including an aerial vehicle in Kinematic mode, which
needs ``vz`` (world-frame vertical rate) the way marine vehicles don't.

There is no thruster/rotor geometry to invert here -- Kinematic has no
physics engine, only a commanded surge speed, yaw rate and vertical rate.
The mapping is a plain gain from force/torque to speed/rate, clamped to
configured limits. It is not a substitute for real thruster or rotor
allocation; it exists so a Kinematic vehicle doesn't need its own
hand-written guidance-that-also-computes-u/w (see ``WaypointFollowerTask``)
just to reuse Navigation/Guidance/Control.
"""

from __future__ import annotations

from geometry_msgs.msg import WrenchStamped
from lotusim_msgs.msg import VesselCmd, VesselCmdArray

from lotusim_sdk.bt.status import Status
from lotusim_sdk.tasks.base import TaskAgent


class KinematicAllocationTask(TaskAgent):
    """Maps a wrench demand to a commanded surge speed, yaw rate and
    vertical rate.

    Params:
        u_gain      float   surge speed per newton of surge force (default 0.05)
        w_gain      float   yaw rate per newton-metre of yaw torque (default 0.05)
        vz_gain     float   vertical rate per newton of heave force (default 0.05)
        u_max_ms    float   commanded surge speed saturation (default 2.0)
        w_max_rads  float   commanded yaw rate saturation (default 1.0)
        vz_max_ms   float   commanded vertical speed saturation (default 1.0)
    """

    PERPETUAL = True

    def __init__(self, host, params=None, blackboard=None, id: str = "") -> None:
        super().__init__(host, params, blackboard, id)
        p = self.params
        self._u_gain = float(p.get("u_gain", 0.05))
        self._w_gain = float(p.get("w_gain", 0.05))
        self._vz_gain = float(p.get("vz_gain", 0.05))
        self._u_max = float(p.get("u_max_ms", 2.0))
        self._w_max = float(p.get("w_max_rads", 1.0))
        self._vz_max = float(p.get("vz_max_ms", 1.0))
        self._sub = None
        self._pub = None

        # Request a kinematic spawn: the host KinematicInterface integrates the
        # {u, w} published by this task. The SDF is emitted at spawn time, AFTER
        # set_missions builds this task but BEFORE the CREATE_CMD is sent, so
        # the flag is in place when PhysicalEntity._lotus_blocks() runs (same
        # mechanism as WaypointFollowerTask).
        host._kinematic_guidance = True

    def on_enter(self) -> None:
        world = self.host.world_name
        agent = self.host.agent_name
        self._pub = self.host.create_publisher(VesselCmdArray, f"/{world}/vessel_cmd_array", 10)
        self._publish(0.0, 0.0, 0.0)
        self._sub = self.host.create_subscription(WrenchStamped, f"/{world}/{agent}/control", self._on_control, 10)

    def on_exit(self, status) -> None:
        if self._sub is not None:
            self.host.destroy_subscription(self._sub)
            self._sub = None
        self._publish(0.0, 0.0, 0.0)

    def update(self) -> Status:
        return Status.RUNNING

    def _on_control(self, msg: WrenchStamped) -> None:
        u = max(-self._u_max, min(self._u_max, self._u_gain * msg.wrench.force.x))
        w = max(-self._w_max, min(self._w_max, self._w_gain * msg.wrench.torque.z))
        # Control's heave is positive-descend (NED-style, see DepthHoldPID);
        # KinematicInterface's vz is world-frame ENU (positive up), hence the sign flip.
        vz = max(-self._vz_max, min(self._vz_max, -self._vz_gain * msg.wrench.force.z))
        self._publish(u, w, vz)

    def _publish(self, u: float, w: float, vz: float) -> None:
        cmd = VesselCmd()
        cmd.vessel_name = self.host.agent_name
        cmd.cmd_string = f'{{"u": {u}, "w": {w}, "vz": {vz}}}'
        array = VesselCmdArray()
        array.cmds = [cmd]
        self._pub.publish(array)
