"""WAMV Control: the vehicle-agnostic ControlTask. WAMV's damping
coefficients have not been characterized, so the current feedforward stays
off (zero) by default -- override ``xu``/``xuu``/``yv``/``yvv`` per
scenario once they are.

Both Kinematic and xdyn Allocation are available -- see
``multi_vehicle_examples/wamv_kinematic.json`` (with
``lotusim_sdk.tasks.kinematic_allocation``) and
``multi_vehicle_examples/wamv_xdyn.json`` (with ``WamvAllocationTask``,
the two stern thrusters of ``assets/models/wamv/wamv.yaml``).

The generic ControlTask saturates the yaw moment at 6 N.m and the surge
force at 141 N, sized for BlueROV. Under xdyn, WAMV's yaw damping
(800 N.m.s/rad plus 800 N.m.s^2/rad^2 in wamv.yaml) needs far more than
6 N.m to turn at all, so
both limits are params here. Their defaults are the generic ones, which
leaves the Kinematic scenario unchanged.

Holding a point: the generic position hold keeps the guidance heading and
asks for a sway force, which WAMV does not have, so a lateral current pushes
it away for good. With ``hold_radius_m`` set, once the vehicle is farther
than that from the hold point it turns its bow to the point and drives back
with surge only, then takes the guidance heading again inside the radius.
"""

from __future__ import annotations

import math

from lotusim_sdk.control import HeadingHoldPID, SurgeSpeedPID, enu_to_ned_position
from lotusim_sdk.tasks.control import ControlTask


class WamvControlTask(ControlTask):
    """See ``ControlTask`` for params. No damping defaults yet for WAMV, so
    feedforward is off unless a scenario supplies xu/xuu/yv/yvv explicitly.

    Extra params:
        n_max_nm  float   yaw moment saturation (default 6, as ControlTask)
        f_max_n   float   surge force saturation in speed mode (default 141,
                          as ControlTask)
        hold_radius_m  float  distance from the hold point beyond which the
                          vehicle points at it (default: off, as ControlTask)
    """

    def __init__(self, host, params=None, blackboard=None, id: str = "") -> None:
        super().__init__(host, params, blackboard, id)
        p = self.params
        self._head = HeadingHoldPID(
            0.0,
            kp=float(p.get("kp_psi", 4.0)),
            ki=float(p.get("ki_psi", 0.1)),
            kd=float(p.get("kd_psi", 2.0)),
            n_max=float(p.get("n_max_nm", 6.0)),
        )
        self._speed = SurgeSpeedPID(
            0.0,
            kp=float(p.get("kp_u", 120.0)),
            ki=float(p.get("ki_u", 20.0)),
            kd=float(p.get("kd_u", 0.0)),
            f_max=float(p.get("f_max_n", 141.0)),
        )
        radius = p.get("hold_radius_m")
        self._hold_radius = None if radius is None else float(radius)

    def _control_step(self) -> None:
        nav, g = self._latest_nav, self._latest_guidance
        if self._hold_radius is not None and nav is not None and g is not None and g.use_position_hold:
            pos = nav.pose.pose.position
            x, y, _ = enu_to_ned_position(pos.x, pos.y, pos.z)
            dx, dy = g.target_x - x, g.target_y - y
            if math.hypot(dx, dy) > self._hold_radius:
                g.desired_heading = math.atan2(dy, dx)
        super()._control_step()
