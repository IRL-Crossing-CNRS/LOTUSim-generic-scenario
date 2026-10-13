#!/usr/bin/env python3
"""Vehicle-agnostic PID building blocks, shared by every LOTUSim agent that
closes its own control loop on the caller side (xdyn vehicles today, any
future non-Kinematic vehicle tomorrow).

None of these classes know about a specific vehicle's thruster layout: they
turn a scalar or planar error into a force/moment request. The mapping from
that request to per-actuator commands (thruster allocation, control surfaces,
...) is vehicle-specific and belongs in the vehicle's own package -- see
``bluerov_gnc/thruster_allocation.py`` for the BlueROV2 Heavy example.

Sign convention here is deliberately NOT fixed to NED/ENU: these classes just
drive an error to zero. The caller picks the sign of its setpoint and error to
match whatever frame it works in (the BlueROV tasks use NED throughout, to
match xdyn).
"""

import math


class PID:
    """A single-axis PID with output clamping and integral anti-windup."""

    def __init__(self, kp, ki, kd, out_min=-1e9, out_max=1e9, i_max=1e9):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.out_min, self.out_max, self.i_max = out_min, out_max, i_max
        self.integral = 0.0
        self.prev_error = None

    def reset(self):
        self.integral = 0.0
        self.prev_error = None

    def update(self, error, dt):
        if dt <= 0.0:
            # No time has elapsed (duplicate timestamp, paused clock): hold
            # the previous output rather than divide by zero on the
            # derivative term or grow the integral on a phantom step.
            return self.kp * error + self.ki * self.integral
        self.integral += error * dt
        # anti-windup: clamp the integral term itself
        if self.ki:
            lim = self.i_max / abs(self.ki)
            self.integral = max(-lim, min(lim, self.integral))
        d = 0.0 if self.prev_error is None else (error - self.prev_error) / dt
        self.prev_error = error
        out = self.kp * error + self.ki * self.integral + self.kd * d
        return max(self.out_min, min(self.out_max, out))


class DepthHoldPID:
    """Hold a depth/immersion setpoint. Output is a heave force, positive in
    whatever direction the caller's convention calls "descend"."""

    def __init__(self, z_setpoint, kp=-120.0, ki=-10.0, kd=-60.0, f_max=100.0):
        self.setpoint = z_setpoint
        self.pid = PID(kp, ki, kd, -f_max, f_max, i_max=f_max)

    def update(self, z, dt):
        # error > 0 means too deep (NED) and must rise -- flip kp/ki/kd for
        # the opposite convention.
        return self.pid.update(z - self.setpoint, dt)


class HeadingHoldPID:
    """Hold a heading setpoint (rad), wrapped to the shortest turn. Output is
    a yaw moment."""

    def __init__(self, psi_setpoint, kp=4.0, ki=0.1, kd=2.0, n_max=6.0):
        self.setpoint = psi_setpoint
        self.pid = PID(kp, ki, kd, -n_max, n_max, i_max=n_max)

    @staticmethod
    def wrap(a):
        return (a + math.pi) % (2 * math.pi) - math.pi

    def update(self, psi, dt):
        return self.pid.update(self.wrap(self.setpoint - psi), dt)


class SurgeSpeedPID:
    """Hold a surge speed setpoint (m/s over ground). Output: surge force."""

    def __init__(self, u_setpoint, kp=120.0, ki=20.0, kd=0.0, f_max=141.0):
        self.setpoint = u_setpoint
        self.pid = PID(kp, ki, kd, -f_max, f_max, i_max=f_max)

    def update(self, u, dt):
        return self.pid.update(self.setpoint - u, dt)


class PositionHoldPID:
    """Hold a fixed point in the horizontal plane.

    Returns a force expressed in the same 2D frame as ``(x_sp, y_sp)``; the
    caller projects it onto its body axes (surge/sway) using the vehicle's
    heading.
    """

    def __init__(self, x_sp, y_sp, kp=60.0, ki=4.0, kd=40.0, f_max=141.0):
        self.x_sp, self.y_sp = x_sp, y_sp
        self.px = PID(kp, ki, kd, -f_max, f_max, i_max=f_max)
        self.py = PID(kp, ki, kd, -f_max, f_max, i_max=f_max)

    def update(self, x, y, dt):
        return self.px.update(self.x_sp - x, dt), self.py.update(self.y_sp - y, dt)
