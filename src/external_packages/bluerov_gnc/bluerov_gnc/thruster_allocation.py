#!/usr/bin/env python3
"""Thruster allocation for the BlueROV2 Heavy xdyn model -- the one piece of
the control stack that is genuinely vehicle-specific (a different thruster
layout needs a different allocator). The generic PID/guidance building blocks
this vehicle's tasks compose around it live in ``lotusim_sdk.control``.

Sign conventions are NED, as everywhere in xdyn: z is positive *downwards*
(z = 2 means 2 m below the surface), psi is positive clockwise seen from above.

Thruster allocation
-------------------
The six `maneuvering` force models of model/bluerov2_heavy.yml give, per unit of T
(the vectored thrusters sit at z = 0, the centre-of-gravity plane, so they
produce no roll/pitch moment):

    prop  position (x,y,z) m        force (X,Y,Z) per N of T
      1   ( 0.156, -0.111, 0.000)   (+0.7071, -0.7071,  0)
      2   ( 0.156,  0.111, 0.000)   (+0.7071, +0.7071,  0)
      3   (-0.156, -0.111, 0.000)   (-0.7071, -0.7071,  0)
      4   (-0.156,  0.111, 0.000)   (-0.7071, +0.7071,  0)
      5   ( 0.120,  0.218, 0.000)   ( 0,       0,      -1)
      8   (-0.120, -0.218, 0.000)   ( 0,       0,      -1)

Inverting that by hand gives four decoupled command patterns:

    surge X : [+1, +1, -1, -1]  ->  X = +2.828 * T
    sway  Y : [+1, -1, +1, -1]  ->  Y = -2.828 * T
    yaw   N : [+1, -1, -1, +1]  ->  N = -0.127 * T
    heave Z : props 5 and 8     ->  Z = -(T5 + T8)

Note the two sign traps: sway and yaw come out *negative* for a positive
pattern, and heave is negative because Z points down in NED while the vertical
thrusters push up.  Both are compensated below so the caller can think in
plain "forward / starboard / down / turn-right" terms.
"""

SQRT2_2 = 0.7071
SURGE_GAIN = 4.0 * SQRT2_2          # N of X per N of T
SWAY_GAIN = 4.0 * SQRT2_2           # N of Y per N of T (sign handled below)
YAW_GAIN = 4.0 * (0.156 * SQRT2_2 - 0.111 * SQRT2_2)  # N.m of N per N of T


class ThrusterAllocator:
    """Map a wrench request (X, Y, Z, N) to the six per-thruster T commands."""

    def __init__(self, t_max=50.0):
        self.t_max = t_max

    def _sat(self, v):
        return max(-self.t_max, min(self.t_max, v))

    def to_commands(self, surge_N, sway_N, heave_N, yaw_Nm):
        """surge>0 forward, sway>0 starboard, heave>0 downward, yaw>0 to starboard."""
        a = surge_N / SURGE_GAIN
        b = -sway_N / SWAY_GAIN     # pattern gives -Y, so flip
        c = -yaw_Nm / YAW_GAIN      # pattern gives -N, so flip
        t1 = self._sat(a + b + c)
        t2 = self._sat(a - b - c)
        t3 = self._sat(-a + b - c)
        t4 = self._sat(-a - b + c)
        # Z in NED points down; the vertical thrusters produce Z = -(T5+T8),
        # so a request to go *down* (heave>0) needs negative T.
        t5 = self._sat(-heave_N / 2.0)
        t8 = self._sat(-heave_N / 2.0)
        return {1: t1, 2: t2, 3: t3, 4: t4, 5: t5, 8: t8}
