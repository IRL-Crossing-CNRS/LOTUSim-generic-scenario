"""Guidance block: turns the navigation state into a ``GuidanceSetpoint``.

Five interchangeable implementations, same input/output topics and messages:

    HoldGuidanceTask                 station keeping: a fixed (x, y, depth, heading)
    LOSGuidanceTask                  line-of-sight tracking of a straight segment
    PurePursuitGuidanceTask          pure-pursuit tracking of the same segment
    LOSPolylineGuidanceTask          line-of-sight tracking of a waypoint polyline
    PurePursuitPolylineGuidanceTask  pure-pursuit tracking of the same polyline

The polyline variants take an explicit waypoint list, so a path whose heading
or depth varies along the way (a horizontal or vertical sinusoid, a survey
lawnmower) is expressed by sampling it into segments; the single-segment
tasks cannot represent one.

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
from lotusim_sdk.control import LineOfSightGuidance, PurePursuitGuidance, enu_to_ned_position


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
        self._pub = self.host.create_publisher(GuidanceSetpoint, f"/{world}/{agent}/guidance", 10)
        self._nav_sub = self.host.create_subscription(Odometry, f"/{world}/{agent}/navigation", self._on_navigation, 10)

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
                f"{type(self).__name__}: holding ({self._x_sp:.2f}, " f"{self._y_sp:.2f}) NED, depth {self._z_sp} m"
            )
        self._publish(
            use_position_hold=True,
            target_x=self._x_sp,
            target_y=self._y_sp,
            desired_heading=self._psi_sp,
            desired_depth=self._z_sp,
            desired_speed=0.0,
            cross_track_error=0.0,
            along_track_distance=0.0,
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
        self._law = None  # built on the first pose: start = current position

    def _on_navigation(self, msg: Odometry) -> None:
        x, y = _ned_xy(msg.pose.pose)
        if self._law is None:
            self._law = self._GUIDANCE_CLS(
                (x, y, self._z_start), (x + self._x_end, y + self._y_end, self._z_end), lookahead=self._lookahead
            )
            self.host.get_logger().info(
                f"{type(self).__name__}: ({x:.1f}, {y:.1f}, {self._law.z1:.1f}) -> "
                f"({self._law.x2:.1f}, {self._law.y2:.1f}, {self._law.z2:.1f}) NED"
            )

        psi_sp, z_sp, cross = self._law.update(x, y)
        along = self._law.along(x, y)

        if along >= self._law.length_h:
            self._publish(
                use_position_hold=True,
                target_x=self._law.x2,
                target_y=self._law.y2,
                desired_heading=self._law.heading,
                desired_depth=self._law.z2,
                desired_speed=0.0,
                cross_track_error=cross,
                along_track_distance=along,
                arrived=True,
            )
        else:
            self._publish(
                use_position_hold=False,
                target_x=0.0,
                target_y=0.0,
                desired_heading=psi_sp,
                desired_depth=z_sp,
                desired_speed=self._u_sp,
                cross_track_error=cross,
                along_track_distance=along,
                arrived=False,
            )


class LOSGuidanceTask(_PathGuidanceTaskBase):
    """Line-of-sight guidance along a straight segment."""

    _GUIDANCE_CLS = LineOfSightGuidance


class PurePursuitGuidanceTask(_PathGuidanceTaskBase):
    """Pure-pursuit guidance along a straight segment."""

    _GUIDANCE_CLS = PurePursuitGuidance


class _PolylineGuidanceTaskBase(_GuidanceTaskBase):
    """Follows a list of 3D waypoints, one straight segment at a time, then
    holds the last one.

    The single-segment tasks above cannot express a path whose heading or
    depth changes along the way. This one takes an explicit waypoint list and
    tracks the active segment with the same guidance law, switching to the
    next segment once the vehicle's along-track distance passes the current
    segment's horizontal length. A curve is therefore expressed as a
    polyline: the caller chooses how finely to sample it.

    Waypoints are given RELATIVE to the first pose received in x/y (so a
    scenario does not need to know where the vehicle spawns) and ABSOLUTE in
    depth, matching ``z_start_m``/``z_end_m`` of the single-segment tasks.
    The first segment runs from the spawn point at ``z_start_m`` to
    waypoint 0.

    ``along_track_distance`` is published CUMULATIVE over the whole path, not
    reset per segment, so a recorded run can be plotted against path distance
    directly. ``cross_track_error`` stays relative to the active segment,
    which is what the vehicle is actually tracking.

    Params:
        waypoints        list    [[dx, dy, z], ...], dx/dy relative to the
                                 start point, z the absolute depth at that
                                 waypoint. Required, at least one entry.
        z_start_m        float   depth at the start point (default 3)
        u_setpoint_ms    float   target surge speed while on the path
                                 (default 0.5)
        lookahead_m      float   guidance lookahead distance (default 5)
    """

    #: Guidance law class, set by the concrete subclass.
    _GUIDANCE_CLS = None

    def __init__(self, host, params=None, blackboard=None, id: str = "") -> None:
        super().__init__(host, params, blackboard, id)
        p = self.params
        wps = p.get("waypoints") or []
        if not wps:
            raise ValueError(
                f"{type(self).__name__}: 'waypoints' is required and must be " "a non-empty list of [dx, dy, z]"
            )
        self._waypoints = [(float(w[0]), float(w[1]), float(w[2])) for w in wps]
        self._z_start = float(p.get("z_start_m", 3.0))
        self._u_sp = float(p.get("u_setpoint_ms", 0.5))
        self._lookahead = float(p.get("lookahead_m", 5.0))
        self._laws = None  # built on the first pose: start = current position
        self._idx = 0  # index of the active segment
        self._done_length = 0.0  # cumulative length of the completed segments

    def _build(self, x: float, y: float) -> None:
        """Build one guidance law per segment, anchored at the start point."""
        pts = [(x, y, self._z_start)]
        pts += [(x + dx, y + dy, z) for dx, dy, z in self._waypoints]
        self._laws = []
        for p1, p2 in zip(pts, pts[1:]):
            # A zero-horizontal-length segment has no heading, so it cannot be
            # tracked: a purely vertical move must be expressed as a shallow
            # ramp instead. Skipping keeps one bad waypoint from killing a run.
            if math.hypot(p2[0] - p1[0], p2[1] - p1[1]) < 1e-6:
                self.host.get_logger().warn(
                    f"{type(self).__name__}: skipping zero-length segment at " f"({p1[0]:.1f}, {p1[1]:.1f})"
                )
                continue
            self._laws.append(self._GUIDANCE_CLS(p1, p2, lookahead=self._lookahead))
        if not self._laws:
            raise ValueError(f"{type(self).__name__}: no trackable segment in 'waypoints'")
        self.host.get_logger().info(
            f"{type(self).__name__}: {len(self._laws)} segment(s), "
            f"({x:.1f}, {y:.1f}, {self._z_start:.1f}) -> "
            f"({self._laws[-1].x2:.1f}, {self._laws[-1].y2:.1f}, "
            f"{self._laws[-1].z2:.1f}) NED, "
            f"{sum(l.length_h for l in self._laws):.1f} m horizontal"
        )

    def _on_navigation(self, msg: Odometry) -> None:
        x, y = _ned_xy(msg.pose.pose)
        if self._laws is None:
            self._build(x, y)

        # Advance through every segment already passed, not just one per
        # sample: a short segment can be overshot within a single navigation
        # period, and stopping at the first would leave guidance behind the
        # vehicle for the rest of the run.
        while self._idx < len(self._laws) - 1:
            law = self._laws[self._idx]
            if law.along(x, y) < law.length_h:
                break
            self._done_length += law.length_h
            self._idx += 1

        law = self._laws[self._idx]
        psi_sp, z_sp, cross = law.update(x, y)
        along = law.along(x, y)
        cumulative = self._done_length + along

        if self._idx == len(self._laws) - 1 and along >= law.length_h:
            self._publish(
                use_position_hold=True,
                target_x=law.x2,
                target_y=law.y2,
                desired_heading=law.heading,
                desired_depth=law.z2,
                desired_speed=0.0,
                cross_track_error=cross,
                along_track_distance=cumulative,
                arrived=True,
            )
        else:
            self._publish(
                use_position_hold=False,
                target_x=0.0,
                target_y=0.0,
                desired_heading=psi_sp,
                desired_depth=z_sp,
                desired_speed=self._u_sp,
                cross_track_error=cross,
                along_track_distance=cumulative,
                arrived=False,
            )


class LOSPolylineGuidanceTask(_PolylineGuidanceTaskBase):
    """Line-of-sight guidance along a waypoint polyline."""

    _GUIDANCE_CLS = LineOfSightGuidance


class PurePursuitPolylineGuidanceTask(_PolylineGuidanceTaskBase):
    """Pure-pursuit guidance along a waypoint polyline."""

    _GUIDANCE_CLS = PurePursuitGuidance
