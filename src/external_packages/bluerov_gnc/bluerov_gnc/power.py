#!/usr/bin/env python3
"""T200 thruster power model, used by the metrics recorder to turn a thrust
history into energy.

The thrusters are not wired to a battery model in the simulator, so energy is
computed from the commanded thrusts instead: nothing here touches the
dynamics.

Model
-----
Propeller momentum theory: P_ideal = T^1.5 / sqrt(2*rho*A). For a T200
(76 mm propeller, A = 4.54e-3 m^2) in sea water (rho = 1025):

    P_ideal = T^1.5 / 3.05

Applying an overall efficiency (propeller + motor + ESC) of ~45 %:

    P = K * |T|^1.5 + P_idle,   K = 0.72 W/N^1.5,  P_idle = 2 W

Against the public Blue Robotics T200 curve at 16 V:

    thrust      9.8 N   19.6 N   29.4 N   39.2 N   51.5 N
    datasheet    22 W     55 W    105 W    170 W    290 W
    model        24 W     64 W    117 W    179 W    268 W
    error      +9.5 %  +17.2 %  +11.2 %   +5.1 %   -7.6 %

The worst point is 17 %, so the absolute values are indicative. Ratios
between conditions run on the same model are unaffected by a constant
calibration error; absolute figures need K refitted against the official
curve first.
"""

K_T200 = 0.72      # W / N^1.5
P_IDLE = 2.0       # W per powered thruster


def thruster_power(thrust_N, k=K_T200, p_idle=P_IDLE):
    """Electrical power of one thruster for a given thrust, in W."""
    return k * abs(thrust_N) ** 1.5 + p_idle


def total_power(thrusts, k=K_T200, p_idle=P_IDLE):
    """Power of the whole thruster set, in W. `thrusts`: iterable of newtons."""
    return sum(thruster_power(t, k, p_idle) for t in thrusts)
