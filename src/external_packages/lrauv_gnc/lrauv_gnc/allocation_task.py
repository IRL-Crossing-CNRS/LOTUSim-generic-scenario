"""Allocation block: wrench -> the single LRAUV propeller command.

Subscribes ``geometry_msgs/WrenchStamped`` on ``/<world>/<agent>/control``
and ``nav_msgs/Odometry`` on ``/<world>/<agent>/navigation``; publishes
``lotusim_msgs/VesselCmdArray`` on ``/<world>/vessel_cmd_array``, same slot
BlueROV's ``ThrusterAllocator`` uses. LRAUV's xdyn model
(``assets/models/lrauv/lrauv.yml``) has exactly one actuator -- a propeller
commanded by ``propeller(rpm)`` and ``propeller(P/D)`` -- and no rudder or
fin model (commented out in the YAML, never enabled), so there is no yaw or
sway actuation to allocate to: only surge is controllable. Guidance's
desired heading/cross-track correction is received by Control but has
nothing downstream to act on it -- LRAUV goes straight once launched, by
design of the vehicle model, not a bug here.

``propeller(rpm)`` is in radians/second despite the name (xdyn's
``AbstractWageningen::advance_ratio`` computes
``n = commands.at("rpm")/(2*PI)``). This class's own params (``rpm_gain``,
``rpm_max``, ``rpm_min``) stay in real RPM, matching LRAUV's own "300 RPM
normal" spec, and are converted to rad/s only at the point of publishing
in ``_publish()``.

The propeller uses an explicit ``Kt(J) & Kq(J)`` table (an MIT-PLL-measured
curve, see that file's own comments). The advance ratio
J = (1-w)*Va/(n*D) is mathematically infinite at n=0 regardless of Va (Va =
the body's surge velocity relative to the current), and this table-based
model refuses the step outright on an infinite/out-of-range J, freezing
the whole scenario. A commanded magnitude below ``rpm_min`` is never sent
for this reason.

A static ``rpm_min`` alone is not enough: J also exceeds the table's
domain when the vehicle's own speed grows (from thrust, current, or both)
faster than a fixed minimum rpm can compensate for. This task subscribes
to Navigation and computes a minimum commanded magnitude from the
vehicle's measured surge speed each control tick:
``rpm_floor_dynamic = |u_measured| / (J_safe*D) * 60``, keeping J inside
its valid domain across the vehicle's operating speed range, not just a
single tuned point.

This propeller's Kt/Kq table has no validated reverse-thrust behaviour
(J's domain is [-0.001, 1.5], barely past zero on the low side):
``AbstractWageningen::advance_ratio`` takes ``Va`` as an absolute value
before the division, so J is negative if and only if the commanded n
itself is negative, independent of the vehicle's actual travel direction.
The commanded magnitude is therefore clamped to [0, rpm_max] rather than
mirroring velocity sign: a negative force demand (braking/reverse) is
handled by commanding rpm down toward its floor, not by reversing the
propeller.

Known limitation: the rpm mapping outside the safety floor is a
proportional gain on force, not a thrust-curve inversion -- it does not
aim for a target J, only avoids leaving the table's valid domain, and has
no way to actively slow down or reverse.
"""

from __future__ import annotations

import json
import math

from geometry_msgs.msg import WrenchStamped
from nav_msgs.msg import Odometry
from lotusim_msgs.msg import VesselCmd, VesselCmdArray

from lotusim_sdk.bt.status import Status
from lotusim_sdk.tasks.base import TaskAgent
from lotusim_sdk.control import enu_to_ned_position, enu_quat_to_ned_euler

#: LRAUV propeller diameter (m) and wake fraction, from lrauv.yml's
#: propeller block (Bellingham et al., "Efficient Propulsion for the
#: Tethys Long-Range AUV", IEEE AUV'2010) -- must match that file for the
#: J estimate here to correspond to what xdyn actually computes.
_PROPELLER_DIAMETER_M = 0.2539
_WAKE_FRACTION = 0.13
_RPM_TO_RAD_S = 2.0 * math.pi / 60.0


class LrauvAllocationTask(TaskAgent):
    """Maps a surge force demand to the propeller's rpm command.

    Params:
        rpm_gain   float   rpm per newton of surge force (default 1.0)
        rpm_max    float   propeller rpm saturation (default 300, LRAUV's own
                            documented "normal" upper value)
        rpm_min    float   static floor on commanded magnitude, applied even
                            with no navigation feedback yet (e.g. the very
                            first published command) -- never exactly 0 (see
                            module docstring for why 0 rpm crashes outright)
                            (default 30)
        j_safe     float   target ceiling for the advance ratio J, used to
                            size the dynamic rpm floor from measured surge
                            speed; kept below the table's real limit (1.5)
                            for margin (default 1.0)
        rpm_slew_max float maximum |rpm| change per control tick (default 5)
        pitch_ratio float  fixed propeller pitch/diameter ratio, P/D
                            (default 0.88, LRAUV's own documented default)
    """

    PERPETUAL = True

    def __init__(self, host, params=None, blackboard=None, id: str = "") -> None:
        super().__init__(host, params, blackboard, id)
        p = self.params
        self._rpm_gain = float(p.get("rpm_gain", 1.0))
        self._rpm_max = float(p.get("rpm_max", 300.0))
        self._rpm_min = float(p.get("rpm_min", 30.0))
        self._j_safe = float(p.get("j_safe", 1.0))
        self._rpm_slew_max = float(p.get("rpm_slew_max", 5.0))
        self._pitch_ratio = float(p.get("pitch_ratio", 0.88))
        self._last_rpm = self._rpm_min
        self._u_measured = 0.0
        self._nav_sub = None
        self._control_sub = None
        self._pub = None

    def on_enter(self) -> None:
        world = self.host.world_name
        agent = self.host.agent_name
        self._pub = self.host.create_publisher(VesselCmdArray, f"/{world}/vessel_cmd_array", 10)
        # xdyn fails every step until it has a complete command set; publish
        # a resting one right away (same reasoning as BlueROV's allocator).
        # Must be rpm_min, not 0: see module docstring, 0 rpm crashes the
        # simulation outright rather than just producing zero thrust.
        self._publish(self._rpm_min)
        self._nav_sub = self.host.create_subscription(Odometry, f"/{world}/{agent}/navigation", self._on_navigation, 10)
        self._control_sub = self.host.create_subscription(
            WrenchStamped, f"/{world}/{agent}/control", self._on_control, 10
        )

    def on_exit(self, status) -> None:
        if self._control_sub is not None:
            self.host.destroy_subscription(self._control_sub)
            self._control_sub = None
        if self._nav_sub is not None:
            self.host.destroy_subscription(self._nav_sub)
            self._nav_sub = None
        self._publish(self._rpm_min)

    def update(self) -> Status:
        return Status.RUNNING

    def _on_navigation(self, msg: Odometry) -> None:
        pose = msg.pose.pose
        _, _, psi = enu_quat_to_ned_euler(
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w
        )
        vx, vy, _ = enu_to_ned_position(msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z)
        self._u_measured = vx * math.cos(psi) + vy * math.sin(psi)

    def _on_control(self, msg: WrenchStamped) -> None:
        # This propeller's Kt/Kq table has no validated negative-J behaviour
        # (domain [-0.001, 1.5] -- see module docstring), so the commanded
        # magnitude is clamped to [0, rpm_max] rather than [-rpm_max,
        # rpm_max]: reverse force demand pulls rpm down toward its floor,
        # it does not reverse the propeller.
        target = max(0.0, min(self._rpm_max, self._rpm_gain * msg.wrench.force.x))

        # Dynamic floor: keep J = (1-w)*|u|/(n*D) <= j_safe for the vehicle's
        # ACTUAL current speed, not just the single speed a static constant
        # was tuned for. n here is in rev/s; the *60 converts back to RPM to
        # match this class's other params. |u| is only a lower-bound estimate
        # of Va = |u-current| (the current itself isn't available here), so
        # j_safe leaves real margin below the table's actual 1.5 limit.
        rpm_floor_dynamic = max(
            self._rpm_min,
            (1.0 - _WAKE_FRACTION) * abs(self._u_measured) / (self._j_safe * _PROPELLER_DIAMETER_M) * 60.0,
        )

        rpm = max(target, rpm_floor_dynamic)
        step = max(-self._rpm_slew_max, min(self._rpm_slew_max, rpm - self._last_rpm))
        self._publish(self._last_rpm + step)

    def _publish(self, rpm: float) -> None:
        self._last_rpm = rpm
        cmd = VesselCmd()
        cmd.vessel_name = self.host.agent_name
        cmd.cmd_string = json.dumps(
            {
                # xdyn expects rad/s here despite the "(rpm)" key name -- see
                # module docstring. rpm (this class's own unit) -> rad/s.
                "propeller(rpm)": rpm * _RPM_TO_RAD_S,
                "propeller(P/D)": self._pitch_ratio,
            }
        )
        array = VesselCmdArray()
        array.cmds = [cmd]
        self._pub.publish(array)
