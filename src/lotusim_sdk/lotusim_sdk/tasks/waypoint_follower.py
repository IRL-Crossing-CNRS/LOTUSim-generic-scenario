from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import List, Optional

from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from lotusim_msgs.msg import VesselCmd, VesselCmdArray

from lotusim_sdk.bt.status import Status
from lotusim_sdk.tasks.base import TaskAgent


@dataclass
class WaypointFollowerConfig:
    guidance_mode: str | None = None
    loop: bool | None = None
    range_tolerance: float | None = None
    linear_accel_limit: float | None = None
    angular_accel_limit: float | None = None
    linear_velocities_limits: tuple[float, float] | None = None
    angular_velocities_limits: float | None = None
    linear_pid: tuple[float, float, float] | None = None
    angular_pid: tuple[float, float, float] | None = None

_EARTH_RADIUS_M = 6_371_000.0


def _normalize_angle(angle: float) -> float:
    """Wrap an angle to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


class WaypointFollowerTask(TaskAgent):
    """Behaviour-tree waypoint-following task — runs on the launching machine.

    Unlike the legacy spawn-time capability (which delegated guidance to the
    host-side Gazebo ``WaypointFollower`` plugin over the ``/<world>/<agent>/waypoints``
    service), this task runs the guidance/control loop **on the agent node
    itself**. It reads the measured pose from ``/<world>/poses`` (fed into
    ``host.current_pose`` by :class:`Entity`), runs a bang-bang or PID
    controller, and publishes a body-frame velocity set-point
    ``{"u": <m/s>, "w": <rad/s>}`` on ``/<world>/vessel_cmd_array``.

    Pose integration happens host-side in the Gazebo ``KinematicInterface``
    (``physics_engine_interface`` plugin) using Gazebo's own simulation time
    step, so there is no host/remote clock divergence and the loop is robust to
    the agent's wall-clock jitter. Because the agent only publishes ROS topics,
    the exact same task + mission JSON runs identically host-side or remote — it
    executes on whichever machine ticks the mission.

    Requirements:
        * The agent must be spawned with ``<connection_type>Kinematic</connection_type>``.
          This task signals that at construction by setting
          ``host._kinematic_guidance = True`` before the SDF is emitted at spawn;
          :class:`PhysicalEntity` then emits the Kinematic block (XDyn disabled).
        * A world geographic origin must be available on the host as
          ``host._world_origin`` (set by ``run_agent`` from ``--origin`` / the
          config ``origin``) so lat/lon waypoints project to the same ENU frame
          Gazebo uses for the poses on ``/<world>/poses``.

    Params (mission JSON ``"params"``):
        waypoints          list of ``{"lat", "lon"}`` dicts (inline). If absent,
                           falls back to ``params["waypoints_file"]`` (patrol
                           file) and finally to ``host.trajectory``.
        waypoints_file     patrol file name resolved via ``ais_pkg``'s
                           ``PatrolFileProvider`` (same as ``print_waypoints``).
        loop               whether to loop over the trajectory (default: the
                           host's ``loop`` attribute, else ``True``).
        control_rate_hz    guidance loop frequency in Hz (default 20.0).
        guidance_mode      "bang_bang" (default) or "pid".
        range_tolerance, linear_accel_limit, angular_accel_limit,
        linear_velocities_limits ([min, max]), angular_velocities_limits,
        angular_pid ([kp, ki, kd])  — controller gains; unset fields fall back
                           to :attr:`DEFAULT_CONFIG` then the controller defaults.
    """

    DEFAULT_CONFIG: WaypointFollowerConfig = WaypointFollowerConfig(
        guidance_mode="bang_bang",
        linear_velocities_limits=(0.0, 15.0),
        angular_accel_limit=0.01,
        angular_velocities_limits=0.05,
    )

    def __init__(self, host, params=None, blackboard=None, id: str = "") -> None:
        super().__init__(host, params, blackboard, id)

        self.trajectory = self._resolve_waypoints()
        self.loop = bool(self.params.get("loop", getattr(host, "loop", True)))
        self.config = self._resolve_config()
        self._control_period = 1.0 / float(self.params.get("control_rate_hz", 20.0))

        # Request a kinematic spawn: the host KinematicInterface integrates the
        # velocity set-point we publish. The SDF is emitted at spawn time, AFTER
        # set_missions builds this task but BEFORE the CREATE_CMD is sent, so the
        # flag is in place when PhysicalEntity._lotus_blocks() runs.
        host._kinematic_guidance = True

        # --- resolved controller parameters (fall back to the C++ defaults) ---
        cfg = self.config
        self._guidance_mode = cfg.guidance_mode or "bang_bang"
        self._range_tolerance = (
            cfg.range_tolerance if cfg.range_tolerance is not None else 0.5
        )
        self._linear_accel_limit = (
            cfg.linear_accel_limit if cfg.linear_accel_limit is not None else 0.5
        )
        self._angular_accel_limit = (
            cfg.angular_accel_limit if cfg.angular_accel_limit is not None else 0.5
        )
        self._v_min, self._v_max = (
            cfg.linear_velocities_limits
            if cfg.linear_velocities_limits is not None
            else (0.0, 5.0)
        )
        self._max_w = (
            cfg.angular_velocities_limits
            if cfg.angular_velocities_limits is not None
            else 1.0
        )
        self._angular_pid = cfg.angular_pid or (0.8, 0.05, 0.4)

        # --- controller state (re-armed in on_enter) ---
        self._cmd_pub = None
        self._control_timer = None
        self._reset_state()

    # ------------------------------------------------------------------
    # Param resolution
    # ------------------------------------------------------------------
    def _resolve_waypoints(self) -> List[dict]:
        if "waypoints" in self.params and isinstance(self.params["waypoints"], list):
            return self.params["waypoints"]
        if "waypoints_file" in self.params:
            from lotusim_sdk.trajectory_providers import PatrolFileProvider

            return PatrolFileProvider(
                self.params["waypoints_file"],
                base_dir=getattr(self.host, "_config_dir", None),
            ).load()
        return list(getattr(self.host, "trajectory", None) or [])

    def _resolve_config(self) -> WaypointFollowerConfig:
        """Build a WaypointFollowerConfig from params, falling back per-field to
        DEFAULT_CONFIG so a partial ``params`` block keeps the tuned defaults."""
        d = self.DEFAULT_CONFIG
        p = self.params

        def vec(key, default):
            val = p.get(key, default)
            return tuple(val) if isinstance(val, (list, tuple)) else val

        return WaypointFollowerConfig(
            guidance_mode=p.get("guidance_mode", d.guidance_mode),
            loop=p.get("loop", d.loop),
            range_tolerance=p.get("range_tolerance", d.range_tolerance),
            linear_accel_limit=p.get("linear_accel_limit", d.linear_accel_limit),
            angular_accel_limit=p.get("angular_accel_limit", d.angular_accel_limit),
            linear_velocities_limits=vec(
                "linear_velocities_limits", d.linear_velocities_limits
            ),
            angular_velocities_limits=p.get(
                "angular_velocities_limits", d.angular_velocities_limits
            ),
            linear_pid=vec("linear_pid", d.linear_pid),
            angular_pid=vec("angular_pid", d.angular_pid),
        )

    def _reset_state(self) -> None:
        self._wp_index = 0
        self._u = 0.0  # commanded forward speed (m/s)
        self._w = 0.0  # commanded yaw rate (rad/s)
        self._heading_integral = 0.0
        self._prev_heading_error = 0.0
        self._prev_yaw = 0.0
        self._distance_error_integral = 0.0
        self._distance_error_previous = 0.0
        self._finished = False
        # Wall-clock timestamp of the previous control step. Used to measure the
        # REAL elapsed dt: the timer is not cadenced reliably (threads are shared
        # between agents under the MultiThreadedExecutor), so assuming a fixed
        # period makes the velocity ramp and the integral terms wrong whenever
        # the loop is starved.
        self._last_step_time: Optional[float] = None
        # Waypoints projected to the world ENU frame (lazily, on first pose).
        self._enu_waypoints: Optional[List[tuple]] = None

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------
    def on_enter(self) -> None:
        self._reset_state()
        if not self.trajectory:
            self.host.get_logger().warning(
                "WaypointFollowerTask: no waypoints provided, guidance inactive."
            )
            return
        world = self.host.world_name
        self._cmd_pub = self.host.create_publisher(
            VesselCmdArray, f"/{world}/vessel_cmd_array", 10
        )
        # Dedicated callback group so the guidance loop is scheduled on its own
        # and cannot be starved behind this node's other callbacks (1 Hz topic
        # discovery, sensor buffering). Critical when several agents share the
        # MultiThreadedExecutor's thread pool.
        self._control_cb_group = MutuallyExclusiveCallbackGroup()
        self._control_timer = self.host.create_timer(
            self._control_period,
            self._control_step,
            callback_group=self._control_cb_group,
        )
        self.host.get_logger().info(
            f"WaypointFollowerTask active: {len(self.trajectory)} waypoints, "
            f"mode={self._guidance_mode}, loop={self.loop}, "
            f"rate={1.0 / self._control_period:.0f} Hz"
        )

    def update(self) -> Status:
        if not self.trajectory:
            return Status.FAILURE
        # A looping trajectory never finishes: stays RUNNING until halted.
        return Status.SUCCESS if self._finished else Status.RUNNING

    def on_exit(self, _status: Status) -> None:
        if self._control_timer is not None:
            self._control_timer.cancel()
            self.host.destroy_timer(self._control_timer)
            self._control_timer = None
        # Send a final full-stop so the host stops integrating motion.
        self._publish_cmd(0.0, 0.0)
        if self._cmd_pub is not None:
            self.host.destroy_publisher(self._cmd_pub)
            self._cmd_pub = None

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------
    def _project_waypoints(self) -> bool:
        """Convert lat/lon waypoints to world ENU coordinates.

        Uses the world geographic origin stored on the host as ``_world_origin``
        (set by run_agent from --origin/config). This is the same projection
        Gazebo uses for its ENU frame, so it is consistent with the poses
        published on /<world>/poses.
        """
        world_origin = getattr(self.host, "_world_origin", None)
        if world_origin is None:
            return False
        lat0, lon0 = world_origin
        cos_lat0 = math.cos(math.radians(lat0))
        self._enu_waypoints = [
            (
                math.radians(float(wp["lon"]) - lon0) * cos_lat0 * _EARTH_RADIUS_M,
                math.radians(float(wp["lat"]) - lat0) * _EARTH_RADIUS_M,
            )
            for wp in self.trajectory
        ]
        return True

    @staticmethod
    def _yaw_from_pose(pose) -> float:
        q = pose.orientation
        return math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )

    def _publish_cmd(self, u: float, w: float) -> None:
        if self._cmd_pub is None:
            return
        cmd = VesselCmd()
        cmd.vessel_name = self.host.agent_name
        cmd.cmd_string = f'{{"u": {u}, "w": {w}}}'
        array = VesselCmdArray()
        array.cmds = [cmd]
        self._cmd_pub.publish(array)

    # ------------------------------------------------------------------
    # Control loop (ported from the legacy WaypointFollowerPlugin, minus the
    # pose integration which now lives host-side in KinematicInterface).
    # ------------------------------------------------------------------
    def _control_step(self) -> None:
        if self._finished:
            return
        if self._enu_waypoints is None and not self._project_waypoints():
            return  # waiting for a world origin
        pose = self.host.current_pose
        if pose is None or not self._enu_waypoints:
            return  # waiting for the first pose feedback

        # Real elapsed time since the previous step, not the nominal period.
        # Clamp it so a scheduler hiccup (timer fired very late) cannot inject a
        # huge dt that would spike the integral/derivative or over-ramp u.
        now = time.monotonic()
        if self._last_step_time is None:
            dt = self._control_period
        else:
            dt = max(1e-3, min(now - self._last_step_time, 0.5))
        self._last_step_time = now

        x = pose.position.x
        y = pose.position.y
        yaw = self._yaw_from_pose(pose)

        if self._wp_index >= len(self._enu_waypoints):
            self._wp_index = 0
        goal_x, goal_y = self._enu_waypoints[self._wp_index]

        dx = goal_x - x
        dy = goal_y - y
        cos_y, sin_y = math.cos(yaw), math.sin(yaw)
        local_x = cos_y * dx + sin_y * dy
        local_y = -sin_y * dx + cos_y * dy
        angle_to_goal = _normalize_angle(math.atan2(local_y, local_x))
        distance_to_goal = math.hypot(local_x, local_y)

        is_last_waypoint = (
            self._wp_index == len(self._enu_waypoints) - 1
        ) and not self.loop

        self._update_linear_velocity(
            distance_to_goal, angle_to_goal, is_last_waypoint, dt
        )
        self._update_angular_velocity(angle_to_goal, goal_x, goal_y, x, y, yaw, dt)

        self._publish_cmd(self._u, self._w)

        # --- goal tracking ---
        if distance_to_goal <= self._range_tolerance:
            self._heading_integral = 0.0
            self._distance_error_integral = 0.0
            if self._wp_index == len(self._enu_waypoints) - 1:
                if self.loop:
                    self._wp_index = 0
                else:
                    self._u = 0.0
                    self._w = 0.0
                    self._publish_cmd(0.0, 0.0)
                    self._finished = True
            else:
                self._wp_index += 1

    def _update_linear_velocity(
        self, distance_to_goal, angle_to_goal, is_last_waypoint, dt
    ) -> None:
        if self._guidance_mode == "bang_bang":
            stopping_distance = (
                self._u ** 2 / 2.0 / self._linear_accel_limit
                if self._linear_accel_limit > 0
                else 0.0
            )
            flag = 0
            if stopping_distance >= distance_to_goal:
                flag = 1 if is_last_waypoint else 0
            elif -1.57 < angle_to_goal < 1.57:
                flag = 2

            # Heading-coupled speed ceiling: slow down when the goal is off the
            # bow so the boat can actually make the turn within its minimum turn
            # radius (u/max_w). Without this, bang_bang holds full speed for any
            # goal within +/-90 deg; arriving even slightly off a waypoint it
            # overshoots and settles into a stable orbit around it (the agent
            # "turning in a loop around a point"). At full alignment cos->1 so
            # top speed is unchanged; abeam (cos->0) it drops to v_min.
            v_ceiling = self._v_min + (self._v_max - self._v_min) * max(
                0.0, math.cos(angle_to_goal)
            )

            vel_change = self._linear_accel_limit * dt
            if flag == 1:
                if self._u < 0:
                    self._u = 0.0
                elif self._u > 0:
                    self._u -= vel_change
            elif flag == 2:
                self._u = min(self._u + vel_change, v_ceiling)
            else:
                self._u = max(self._u - vel_change, self._v_min)
        else:
            distance_error = distance_to_goal
            self._distance_error_integral += distance_error * dt
            max_integral_contribution = 0.2 * self._v_max
            ki = 0.05
            max_integral = max_integral_contribution / ki
            self._distance_error_integral = max(
                -max_integral, min(self._distance_error_integral, max_integral)
            )
            distance_error_derivative = (
                distance_error - self._distance_error_previous
            ) / dt
            self._distance_error_previous = distance_error

            kp, kd = 0.5, 0.1
            desired_velocity = (
                kp * distance_error
                + ki * self._distance_error_integral
                + kd * distance_error_derivative
            )
            angle_factor = max(0.0, math.cos(angle_to_goal))
            desired_velocity *= angle_factor
            desired_velocity = max(
                self._v_min, min(desired_velocity, self._v_max)
            )
            if is_last_waypoint and distance_to_goal < 0.1:
                desired_velocity = 0.0

            velocity_change = desired_velocity - self._u
            max_accel = self._linear_accel_limit * dt
            velocity_change = max(-max_accel, min(velocity_change, max_accel))
            self._u = self._u + velocity_change
            self._u = max(self._v_min, min(self._u, self._v_max))

    def _update_angular_velocity(
        self, angle_to_goal, goal_x, goal_y, x, y, yaw, dt
    ) -> None:
        kp, ki, kd = self._angular_pid
        max_w = self._max_w

        if self._guidance_mode == "bang_bang":
            desired_yaw = math.atan2(goal_y - y, goal_x - x)
            heading_error = _normalize_angle(desired_yaw - yaw)
            d_error = (heading_error - self._prev_heading_error) / dt
            self._prev_heading_error = heading_error

            # Integrate the heading error, then clamp the accumulator so the
            # integral term can never exceed its share of max_w. Without this
            # anti-windup the integral grows unbounded while the agent turns in
            # place toward a waypoint behind it; w then stays saturated long
            # after the heading is reached, overshooting into a full rotation
            # (the spirals/loops). Mirrors the PID branch below.
            self._heading_integral += heading_error * dt
            if ki > 1e-6:
                max_integral_contribution = 0.5 * max_w
                integral_max = max_integral_contribution / ki
                self._heading_integral = max(
                    -integral_max, min(self._heading_integral, integral_max)
                )
            desired_w = (
                kp * heading_error
                + ki * self._heading_integral
                + kd * d_error
            )
            desired_w = max(-max_w, min(desired_w, max_w))

            # Bleed the integral down once we are pointing roughly at the goal
            # so residual wind-up cannot push the heading past the target.
            if abs(heading_error) < 0.05:
                self._heading_integral *= 0.9
        else:
            heading_error = angle_to_goal
            max_integral_contribution = 0.2 * max_w
            self._heading_integral += heading_error * dt
            if ki > 1e-6:
                integral_max = max_integral_contribution / ki
                self._heading_integral = max(
                    -integral_max, min(self._heading_integral, integral_max)
                )
            yaw_diff = _normalize_angle(yaw - self._prev_yaw)
            current_yaw_rate = yaw_diff / dt
            self._prev_yaw = yaw
            derivative_term = -kd * current_yaw_rate
            desired_w = (
                kp * heading_error
                + ki * self._heading_integral
                + derivative_term
            )
            unclamped_w = desired_w
            desired_w = max(-max_w, min(desired_w, max_w))
            if ki > 1e-6 and abs(unclamped_w) > max_w:
                self._heading_integral = (
                    desired_w - kp * heading_error - derivative_term
                ) / ki
            if abs(heading_error) < 0.05:
                self._heading_integral *= 0.95

        # acceleration limit on the yaw rate
        vel_change = self._angular_accel_limit * dt
        if self._w < desired_w:
            self._w = min(self._w + vel_change, desired_w)
        else:
            self._w = max(self._w - vel_change, desired_w)
