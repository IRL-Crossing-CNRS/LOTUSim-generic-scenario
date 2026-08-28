"""Guidance block: turns the navigation state into a ``GuidanceSetpoint``.

Three interchangeable implementations, same input/output topics and messages:

    HoldGuidanceTask           station keeping: a fixed (x, y, depth, heading)
    LOSGuidanceTask            line-of-sight tracking of a straight segment
    PurePursuitGuidanceTask    pure-pursuit tracking of the same segment

Changing the mission JSON's guidance task name is enough to switch between
them; Navigation, Control and Allocation do not change. Vehicle-agnostic:
any vehicle class can register these directly, or subclass one if it needs
different guidance logic.
"""

from __future__ import annotations

import math

from nav_msgs.msg import Odometry
from lotusim_msgs.msg import GuidanceSetpoint

from lotusim_sdk.bt.status import Status
from lotusim_sdk.tasks.base import TaskAgent
from lotusim_sdk.control import (LineOfSightGuidance, PurePursuitGuidance,
                                  enu_to_ned_position)


def _ned_xy(pose) -> tuple:
    x, y, _ = enu_to_ned_position(pose.position.x, pose.position.y, pose.position.z)
    return x, y


class _GuidanceTaskBase(TaskAgent):
    """Common bookkeeping: subscribe navigation, publish guidance setpoints."""

    PERPETUAL = True

    def __init__(self, host, params=None, blackboard=None, id: str = "") -> None:
        super().__init__(host, params, blackboard, id)
        self._nav_sub = None
        self._pub = None

    def on_enter(self) -> None:
        world = self.host.world_name
        agent = self.host.agent_name
        self._pub = self.host.create_publisher(
            GuidanceSetpoint, f"/{world}/{agent}/guidance", 10)
        self._nav_sub = self.host.create_subscription(
            Odometry, f"/{world}/{agent}/navigation", self._on_navigation, 10)

    def on_exit(self, status) -> None:
        if self._nav_sub is not None:
            self.host.destroy_subscription(self._nav_sub)
            self._nav_sub = None
        if self._pub is not None:
            self.host.destroy_publisher(self._pub)
            self._pub = None

    def update(self) -> Status:
        return Status.RUNNING

    def _on_navigation(self, msg: Odometry) -> None:
        raise NotImplementedError

    def _publish(self, **fields) -> None:
        msg = GuidanceSetpoint(**fields)
        msg.header.stamp = self.host.get_clock().now().to_msg()
        self._pub.publish(msg)


class HoldGuidanceTask(_GuidanceTaskBase):
    """Hold a fixed point in the horizontal plane, depth and heading.

    Params:
        z_setpoint_m      float   target depth, positive down (default 3)
        psi_setpoint_deg  float   target heading (default 0)
        hold_origin       bool    True = hold the position of the first pose
                                  received; otherwise x/y_setpoint_m (default True)
        x_setpoint_m,
        y_setpoint_m      float   target point when hold_origin is False
    """

    def __init__(self, host, params=None, blackboard=None, id: str = "") -> None:
        super().__init__(host, params, blackboard, id)
        p = self.params
        self._hold_origin = bool(p.get("hold_origin", True))
        self._x_sp = float(p.get("x_setpoint_m", 0.0))
        self._y_sp = float(p.get("y_setpoint_m", 0.0))
        self._z_sp = float(p.get("z_setpoint_m", 3.0))
        self._psi_sp = math.radians(float(p.get("psi_setpoint_deg", 0.0)))
        self._initialized = False

    def _on_navigation(self, msg: Odometry) -> None:
        x, y = _ned_xy(msg.pose.pose)
        if not self._initialized:
            if self._hold_origin:
                self._x_sp, self._y_sp = x, y
            self._initialized = True
            self.host.get_logger().info(
                f"{type(self).__name__}: holding ({self._x_sp:.2f}, "
                f"{self._y_sp:.2f}) NED, depth {self._z_sp} m")
        self._publish(
            use_position_hold=True,
            target_x=self._x_sp, target_y=self._y_sp,
            desired_heading=self._psi_sp, desired_depth=self._z_sp,
            desired_speed=0.0, cross_track_error=0.0, along_track_distance=0.0,
            arrived=True,
        )


class _PathGuidanceTaskBase(_GuidanceTaskBase):
    """Shared segment-tracking logic: follows a straight segment from the
    first pose received to a fixed end point, then holds the end point.

    Params:
        z_start_m, z_end_m   float   depth at the start and at the end
        x_end_m, y_end_m     float   segment end, relative to the start point
                                     (default 200, 0)
        u_setpoint_ms        float   target surge speed while on the segment
                                     (default 0.5)
        lookahead_m          float   guidance lookahead distance (default 5)
    """

    #: Guidance law class, set by the concrete subclass.
    _GUIDANCE_CLS = None

    def __init__(self, host, params=None, blackboard=None, id: str = "") -> None:
        super().__init__(host, params, blackboard, id)
        p = self.params
        self._z_start = float(p.get("z_start_m", 3.0))
        self._z_end = float(p.get("z_end_m", 55.0))
        self._x_end = float(p.get("x_end_m", 200.0))
        self._y_end = float(p.get("y_end_m", 0.0))
        self._u_sp = float(p.get("u_setpoint_ms", 0.5))
        self._lookahead = float(p.get("lookahead_m", 5.0))
        self._law = None    # built on the first pose: start = current position

    def _on_navigation(self, msg: Odometry) -> None:
        x, y = _ned_xy(msg.pose.pose)
        if self._law is None:
            self._law = self._GUIDANCE_CLS(
                (x, y, self._z_start),
                (x + self._x_end, y + self._y_end, self._z_end),
                lookahead=self._lookahead)
            self.host.get_logger().info(
                f"{type(self).__name__}: ({x:.1f}, {y:.1f}, {self._law.z1:.1f}) -> "
                f"({self._law.x2:.1f}, {self._law.y2:.1f}, {self._law.z2:.1f}) NED")

        psi_sp, z_sp, cross = self._law.update(x, y)
        along = self._law.along(x, y)

        if along >= self._law.length_h:
            self._publish(
                use_position_hold=True,
                target_x=self._law.x2, target_y=self._law.y2,
                desired_heading=self._law.heading, desired_depth=self._law.z2,
                desired_speed=0.0, cross_track_error=cross,
                along_track_distance=along, arrived=True,
            )
        else:
            self._publish(
                use_position_hold=False,
                target_x=0.0, target_y=0.0,
                desired_heading=psi_sp, desired_depth=z_sp,
                desired_speed=self._u_sp, cross_track_error=cross,
                along_track_distance=along, arrived=False,
            )


class LOSGuidanceTask(_PathGuidanceTaskBase):
    """Line-of-sight guidance along a straight segment."""

    _GUIDANCE_CLS = LineOfSightGuidance


class PurePursuitGuidanceTask(_PathGuidanceTaskBase):
    """Pure-pursuit guidance along a straight segment."""

    _GUIDANCE_CLS = PurePursuitGuidance
