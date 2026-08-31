"""Control block: PID. Turns (navigation, guidance) into a body-frame wrench.

Subscribes ``nav_msgs/Odometry`` on ``/<world>/<agent>/navigation`` and
``lotusim_msgs/GuidanceSetpoint`` on ``/<world>/<agent>/guidance``; publishes
``geometry_msgs/WrenchStamped`` on ``/<world>/<agent>/control``. Reads
``use_position_hold`` off the guidance message rather than being written
per-mission, so the same node serves station keeping, LOS tracking and
pure-pursuit tracking alike.

Vehicle-agnostic: the PID gains are all params, and the damping used to
size the optional current feedforward defaults to zero here (no
compensation) -- a vehicle-specific subclass sets ``DEFAULT_XU`` /
``DEFAULT_XUU`` / ``DEFAULT_YV`` / ``DEFAULT_YVV`` to its own hydrodynamic
damping (from its xdyn YAML) so a scenario doesn't have to pass them every
time.
"""

from __future__ import annotations

import math

from geometry_msgs.msg import WrenchStamped
from nav_msgs.msg import Odometry
from lotusim_msgs.msg import GuidanceSetpoint

from lotusim_sdk.bt.status import Status
from lotusim_sdk.tasks.base import TaskAgent
from lotusim_sdk.control import (
    DepthHoldPID,
    HeadingHoldPID,
    PositionHoldPID,
    SurgeSpeedPID,
    build_feedforward,
    enu_quat_to_ned_euler,
    enu_to_ned_position,
)


class ControlTask(TaskAgent):
    """PID control law: (navigation, guidance) -> body-frame wrench.

    Optionally adds a model-based current feedforward on the horizontal axes
    (see ``lotusim_sdk.control.current_feedforward``). With feedforward off
    -- the default -- this task behaves exactly as before, so the
    pure-feedback baseline is unchanged.

    Params:
        control_rate_hz  float   loop rate (default 20)
        kp_z, ki_z, kd_z,
        kp_psi, ki_psi, kd_psi,
        kp_xy, ki_xy, kd_xy,
        kp_u, ki_u, kd_u         gains -- keep them IDENTICAL across current
                                 conditions, otherwise a comparison between
                                 conditions is void
        feedforward      dict    {"model": "none"|"uniform"|"ekman", ...} --
                                 the current model the CONTROLLER believes in,
                                 which is independent of the current the
                                 SCENARIO actually applies. Keep the applied
                                 current fixed and vary only this to attribute
                                 a difference to the model itself.
        ff_gain          float   scales the feedforward (default 1.0)
        xu, xuu, yv, yvv float   damping used to size the feedforward
                                 (default: this class's DEFAULT_XU etc.)
    """

    PERPETUAL = True

    #: Horizontal damping defaults for the feedforward, overridden by a
    #: vehicle-specific subclass. Zero here means "no compensation" unless a
    #: scenario or subclass supplies real values.
    DEFAULT_XU = 0.0
    DEFAULT_XUU = 0.0
    DEFAULT_YV = 0.0
    DEFAULT_YVV = 0.0

    def __init__(self, host, params=None, blackboard=None, id: str = "") -> None:
        super().__init__(host, params, blackboard, id)
        p = self.params
        self._rate = float(p.get("control_rate_hz", 20.0))
        self._dt = 1.0 / self._rate

        self._depth = DepthHoldPID(
            0.0, kp=float(p.get("kp_z", -120.0)), ki=float(p.get("ki_z", -10.0)), kd=float(p.get("kd_z", -60.0))
        )
        self._head = HeadingHoldPID(
            0.0, kp=float(p.get("kp_psi", 4.0)), ki=float(p.get("ki_psi", 0.1)), kd=float(p.get("kd_psi", 2.0))
        )
        self._pos = PositionHoldPID(
            0.0, 0.0, kp=float(p.get("kp_xy", 60.0)), ki=float(p.get("ki_xy", 4.0)), kd=float(p.get("kd_xy", 40.0))
        )
        self._speed = SurgeSpeedPID(
            0.0, kp=float(p.get("kp_u", 120.0)), ki=float(p.get("ki_u", 20.0)), kd=float(p.get("kd_u", 0.0))
        )
        self._ff = build_feedforward(
            p.get("feedforward"),
            xu=float(p.get("xu", self.DEFAULT_XU)),
            xuu=float(p.get("xuu", self.DEFAULT_XUU)),
            yv=float(p.get("yv", self.DEFAULT_YV)),
            yvv=float(p.get("yvv", self.DEFAULT_YVV)),
            gain=float(p.get("ff_gain", 1.0)),
        )

        self._nav_sub = None
        self._guidance_sub = None
        self._pub = None
        self._timer = None
        self._latest_nav = None
        self._latest_guidance = None

    def on_enter(self) -> None:
        world = self.host.world_name
        agent = self.host.agent_name
        self._pub = self.host.create_publisher(WrenchStamped, f"/{world}/{agent}/control", 10)
        self._nav_sub = self.host.create_subscription(Odometry, f"/{world}/{agent}/navigation", self._on_navigation, 10)
        self._guidance_sub = self.host.create_subscription(
            GuidanceSetpoint, f"/{world}/{agent}/guidance", self._on_guidance, 10
        )
        self._timer = self.host.create_timer(self._dt, self._control_step)

    def on_exit(self, status) -> None:
        if self._timer is not None:
            self._timer.cancel()
            self._timer = None

    def update(self) -> Status:
        return Status.RUNNING

    def _on_navigation(self, msg: Odometry) -> None:
        self._latest_nav = msg

    def _on_guidance(self, msg: GuidanceSetpoint) -> None:
        self._latest_guidance = msg

    def _control_step(self) -> None:
        if self._latest_nav is None or self._latest_guidance is None:
            return  # waiting for both inputs
        nav, g = self._latest_nav, self._latest_guidance

        pose = nav.pose.pose
        x, y, z = enu_to_ned_position(pose.position.x, pose.position.y, pose.position.z)
        _, _, psi = enu_quat_to_ned_euler(
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w
        )
        vx, vy, _ = enu_to_ned_position(nav.twist.twist.linear.x, nav.twist.twist.linear.y, nav.twist.twist.linear.z)
        u = vx * math.cos(psi) + vy * math.sin(psi)
        v = -vx * math.sin(psi) + vy * math.cos(psi)

        self._depth.setpoint = g.desired_depth
        self._head.setpoint = g.desired_heading
        heave = self._depth.update(z, self._dt)
        yaw = self._head.update(psi, self._dt)

        if g.use_position_hold:
            self._pos.x_sp, self._pos.y_sp = g.target_x, g.target_y
            fx, fy = self._pos.update(x, y, self._dt)
            c, s = math.cos(psi), math.sin(psi)
            surge = fx * c + fy * s
            sway = -fx * s + fy * c
        else:
            self._speed.setpoint = g.desired_speed
            surge = self._speed.update(u, self._dt)
            sway = 0.0

        # Model-based current feedforward, added to the feedback demand on the
        # horizontal axes only (the current model is horizontal). `z` is NED,
        # positive down, so it is the depth the model is queried at. Zero
        # unless a scenario configures a model, leaving the baseline unchanged.
        ff_surge, ff_sway = self._ff.wrench(z, psi, u, v)
        surge += ff_surge
        sway += ff_sway

        wrench = WrenchStamped()
        wrench.header.stamp = self.host.get_clock().now().to_msg()
        wrench.wrench.force.x = surge
        wrench.wrench.force.y = sway
        wrench.wrench.force.z = heave
        wrench.wrench.torque.z = yaw
        self._pub.publish(wrench)
