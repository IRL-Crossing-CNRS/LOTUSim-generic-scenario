"""Navigation block: publishes the vehicle's state estimate.

Publishes ``nav_msgs/Odometry`` on ``/<world>/<agent>/navigation``, at
``control_rate_hz``. This implementation is a passthrough of the simulator's
ground-truth pose (``host.current_pose``/``current_twist``): no sensor model,
no estimation error. A different Navigation implementation (an EKF fusing
IMU/DVL/pressure) publishes the same message on the same topic; Guidance and
Control do not change.

Vehicle-agnostic: any vehicle class can register this task directly under
the ``navigation`` entry point, or subclass it if it needs a different
state estimator.
"""

from __future__ import annotations

from nav_msgs.msg import Odometry
from rclpy.time import Time

from lotusim_sdk.bt.status import Status
from lotusim_sdk.tasks.base import TaskAgent


class NavigationTask(TaskAgent):
    """Publishes ``nav_msgs/Odometry`` from the simulator's ground-truth pose.

    Params:
        control_rate_hz  float   publish rate (default 20)
    """

    PERPETUAL = True

    def __init__(self, host, params=None, blackboard=None, id: str = "") -> None:
        super().__init__(host, params, blackboard, id)
        self._rate = float(self.params.get("control_rate_hz", 20.0))
        self._pub = None
        self._timer = None
        # World-frame (ENU) velocity fallback when the simulator does not
        # publish twist: exponentially-smoothed finite difference of position.
        self._prev_t = None
        self._prev_x = None
        self._prev_y = None
        self._vx_est = 0.0
        self._vy_est = 0.0

    def on_enter(self) -> None:
        world = self.host.world_name
        agent = self.host.agent_name
        # Kept across re-activations; re-creating it on every on_enter would
        # strand the previous publisher on the node.
        if self._pub is None:
            self._pub = self.host.create_publisher(
                Odometry, f"/{world}/{agent}/navigation", 10)
        self._timer = self.host.create_timer(
            1.0 / self._rate, self._tick, callback_group=self.callback_group)

    def on_exit(self, status) -> None:
        # cancel() stops the timer firing but leaves it registered on the node
        # and in the executor's wait set, so repeated activations make every
        # spin progressively more expensive.
        if self._timer is not None:
            self._timer.cancel()
            self.host.destroy_timer(self._timer)
            self._timer = None

    def update(self) -> Status:
        return Status.RUNNING

    def _tick(self) -> None:
        pose = self.host.current_pose
        t = self.host.current_pose_sim_time
        if pose is None or t is None:
            return

        twist = self.host.current_twist
        if twist is not None:
            vx, vy, vz = twist.linear.x, twist.linear.y, twist.linear.z
            wx, wy, wz = twist.angular.x, twist.angular.y, twist.angular.z
        else:
            vz = wx = wy = wz = 0.0
            if self._prev_t is not None:
                dt = t - self._prev_t
                if dt > 1e-6:
                    vx_raw = (pose.position.x - self._prev_x) / dt
                    vy_raw = (pose.position.y - self._prev_y) / dt
                    self._vx_est += 0.3 * (vx_raw - self._vx_est)
                    self._vy_est += 0.3 * (vy_raw - self._vy_est)
            vx, vy = self._vx_est, self._vy_est
        self._prev_t, self._prev_x, self._prev_y = t, pose.position.x, pose.position.y

        msg = Odometry()
        # The simulation time the pose is FROM, not the time this ran. The node
        # clock is wall time here, so it advances between two ticks that read
        # the same pose — the world only produces a new one per physics step.
        # A consumer differentiating the state then divides a zero change by a
        # non-zero dt; that is what made the heading loop hunt. Same convention
        # as WaypointFollowerTask, which already works off the poses' stamp.
        msg.header.stamp = Time(seconds=t).to_msg()
        msg.header.frame_id = self.host.world_name
        msg.child_frame_id = self.host.agent_name
        msg.pose.pose = pose
        msg.twist.twist.linear.x = vx
        msg.twist.twist.linear.y = vy
        msg.twist.twist.linear.z = vz
        msg.twist.twist.angular.x = wx
        msg.twist.twist.angular.y = wy
        msg.twist.twist.angular.z = wz
        self._pub.publish(msg)
