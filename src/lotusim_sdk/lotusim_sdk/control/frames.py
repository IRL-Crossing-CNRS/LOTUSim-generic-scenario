#!/usr/bin/env python3
"""ENU (Gazebo) <-> NED (xdyn) frame conversions.

Any xdyn-driven vehicle reads its pose from ``host.current_pose`` in ENU
(Gazebo's convention) and needs it in NED to talk to xdyn or to run a control
law written in NED, so this is not BlueROV-specific -- every future xdyn
vehicle needs the exact same conversion.
"""

import math


def enu_to_ned_position(x, y, z):
    """Same convention as ``lotusim::gazebo::vecEnuToNed()``."""
    return y, x, -z


def world_to_body(vx, vy, vz, qx, qy, qz, qw):
    """A world-frame vector expressed in the body frame of orientation q.

    With an ENU world and a FLU body (Gazebo's conventions) this turns a
    world-frame velocity into the body-frame twist of ``nav_msgs/Odometry``
    (REP 105: the twist is given in ``child_frame_id``). Rotation by q^-1.
    """
    # Rotation by the conjugate (w, -r): v' = v + 2w (c x v) + 2 c x (c x v), c = -r.
    cx, cy, cz = -qx, -qy, -qz
    tx, ty, tz = cy * vz - cz * vy, cz * vx - cx * vz, cx * vy - cy * vx
    ux, uy, uz = cy * tz - cz * ty, cz * tx - cx * tz, cx * ty - cy * tx
    return vx + 2.0 * (qw * tx + ux), vy + 2.0 * (qw * ty + uy), vz + 2.0 * (qw * tz + uz)


def body_flu_to_frd(x, y, z):
    """A body-frame vector from FLU (Gazebo, ROS) to FRD (xdyn, marine)."""
    return x, -y, -z


def enu_quat_to_ned_euler(qx, qy, qz, qw):
    """ENU quaternion (Gazebo) -> (phi, theta, psi) in NED.

    q_ned = q_ned_to_enu^-1 * q_enu * q_frd_to_flu, the inverse of
    ``lotusim::gazebo::quatNedToEnu()``. The world basis change
    (q_ned_to_enu, 180 deg about (1,1,0)/sqrt2) and the body one
    (q_frd_to_flu, 180 deg about x) are different rotations, so this is not a
    conjugation -- see ``xdyn_websocket.cpp``. Both sides must change together.
    """
    s = 0.70710678118
    b = (0.0, 1.0, 0.0, 0.0)  # q_frd_to_flu, (w, x, y, z)
    rinv = (0.0, -s, -s, 0.0)  # q_ned_to_enu^-1

    def mul(a, b):
        aw, ax, ay, az = a
        bw, bx, by, bz = b
        return (
            aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
        )

    qw_, qx_, qy_, qz_ = mul(mul(rinv, (qw, qx, qy, qz)), b)
    sinr = 2.0 * (qw_ * qx_ + qy_ * qz_)
    cosr = 1.0 - 2.0 * (qx_ * qx_ + qy_ * qy_)
    phi = math.atan2(sinr, cosr)
    sinp = max(-1.0, min(1.0, 2.0 * (qw_ * qy_ - qz_ * qx_)))
    theta = math.asin(sinp)
    siny = 2.0 * (qw_ * qz_ + qx_ * qy_)
    cosy = 1.0 - 2.0 * (qy_ * qy_ + qz_ * qz_)
    psi = math.atan2(siny, cosy)
    return phi, theta, psi
