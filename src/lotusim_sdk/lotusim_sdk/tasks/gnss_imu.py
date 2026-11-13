"""Noisy position fix and IMU, for state estimators that replace the navigation block.

The reference navigation block (``navigation``) republishes the simulator's
ground truth. A state estimator that replaces it needs measurements instead.
This task makes two from the ground truth, with seeded Gaussian noise:

- ``/<world>/<agent>/gnss`` (``geometry_msgs/PoseWithCovarianceStamped``): the
  horizontal position in the world frame (ENU, metres, ``frame_id`` = world),
  as a GNSS receiver gives it once converted to local coordinates, at
  ``gnss_rate_hz`` with standard deviation ``gnss_sigma_m`` per axis;
- ``/<world>/<agent>/imu`` (``sensor_msgs/Imu``, ``frame_id`` = agent): the
  orientation (heading noise ``heading_sigma_deg``, roll and pitch noise
  ``tilt_sigma_deg``) and the body-frame angular velocity (noise
  ``gyro_sigma_rad_s``) at ``imu_rate_hz``. Linear acceleration is not
  modelled: its covariance is marked unknown (first element -1, as
  sensor_msgs/Imu specifies).

Covariances are the noise variances, so a filter weighs the measurements as
they are. Stamps are the node clock, like an estimator running beside it.
"""

from __future__ import annotations

import math
import random

from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import Imu

from lotusim_sdk.bt.status import Status
from lotusim_sdk.control.frames import world_to_body
from lotusim_sdk.tasks.base import TaskAgent


def _quat_mul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def _small_rotation(rx, ry, rz):
    """Quaternion (x, y, z, w) of a small rotation vector (rad)."""
    angle = math.sqrt(rx * rx + ry * ry + rz * rz)
    if angle < 1e-12:
        return (0.0, 0.0, 0.0, 1.0)
    s = math.sin(angle / 2.0) / angle
    return (rx * s, ry * s, rz * s, math.cos(angle / 2.0))


class GnssImuTask(TaskAgent):
    """Publishes a noisy position fix and a noisy IMU from the ground truth.

    Params:
        gnss_rate_hz       float  position fix rate (default 5)
        gnss_sigma_m       float  horizontal noise per axis, m (default 1.0)
        imu_rate_hz        float  IMU rate (default 50)
        heading_sigma_deg  float  heading noise, deg (default 1.0)
        tilt_sigma_deg     float  roll and pitch noise, deg (default 0.5)
        gyro_sigma_rad_s   float  angular velocity noise, rad/s (default 0.005)
        seed               int    seed of the noise (default 0)
    """

    PERPETUAL = True

    def __init__(self, host, params=None, blackboard=None, id: str = "") -> None:
        super().__init__(host, params, blackboard, id)
        p = self.params
        self._gnss_rate = float(p.get("gnss_rate_hz", 5.0))
        self._gnss_sigma = float(p.get("gnss_sigma_m", 1.0))
        self._imu_rate = float(p.get("imu_rate_hz", 50.0))
        self._heading_sigma = math.radians(float(p.get("heading_sigma_deg", 1.0)))
        self._tilt_sigma = math.radians(float(p.get("tilt_sigma_deg", 0.5)))
        self._gyro_sigma = float(p.get("gyro_sigma_rad_s", 0.005))
        self._rng = random.Random(int(p.get("seed", 0)))
        self._gnss_pub = self._imu_pub = None
        self._timers = []

    def on_enter(self) -> None:
        world, agent = self.host.world_name, self.host.agent_name
        if self._gnss_pub is None:
            self._gnss_pub = self.host.create_publisher(PoseWithCovarianceStamped, f"/{world}/{agent}/gnss", 10)
            self._imu_pub = self.host.create_publisher(Imu, f"/{world}/{agent}/imu", 10)
        self._timers = [
            self.host.create_timer(1.0 / self._gnss_rate, self._gnss_tick, callback_group=self.callback_group),
            self.host.create_timer(1.0 / self._imu_rate, self._imu_tick, callback_group=self.callback_group),
        ]

    def on_exit(self, status) -> None:
        for timer in self._timers:
            timer.cancel()
            self.host.destroy_timer(timer)
        self._timers = []

    def update(self) -> Status:
        return Status.RUNNING

    def _gnss_tick(self) -> None:
        pose = self.host.current_pose
        if pose is None:
            return
        g = self._rng.gauss
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.host.get_clock().now().to_msg()
        msg.header.frame_id = self.host.world_name
        msg.pose.pose.position.x = pose.position.x + g(0.0, self._gnss_sigma)
        msg.pose.pose.position.y = pose.position.y + g(0.0, self._gnss_sigma)
        msg.pose.pose.position.z = pose.position.z
        msg.pose.pose.orientation.w = 1.0
        cov = [0.0] * 36
        cov[0] = cov[7] = self._gnss_sigma**2
        cov[14] = cov[21] = cov[28] = cov[35] = 1e6  # z and attitude not measured
        msg.pose.covariance = cov
        self._gnss_pub.publish(msg)

    def _imu_tick(self) -> None:
        pose = self.host.current_pose
        if pose is None:
            return
        g = self._rng.gauss
        q = pose.orientation
        # Orientation error as a small rotation in the body frame.
        err = _small_rotation(g(0.0, self._tilt_sigma), g(0.0, self._tilt_sigma), g(0.0, self._heading_sigma))
        qx, qy, qz, qw = _quat_mul((q.x, q.y, q.z, q.w), err)
        twist = self.host.current_twist
        wx, wy, wz = (0.0, 0.0, 0.0)
        if twist is not None:
            wx, wy, wz = world_to_body(twist.angular.x, twist.angular.y, twist.angular.z, q.x, q.y, q.z, q.w)

        msg = Imu()
        msg.header.stamp = self.host.get_clock().now().to_msg()
        msg.header.frame_id = self.host.agent_name
        msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w = qx, qy, qz, qw
        msg.orientation_covariance = [
            self._tilt_sigma**2, 0.0, 0.0,
            0.0, self._tilt_sigma**2, 0.0,
            0.0, 0.0, self._heading_sigma**2,
        ]
        msg.angular_velocity.x = wx + g(0.0, self._gyro_sigma)
        msg.angular_velocity.y = wy + g(0.0, self._gyro_sigma)
        msg.angular_velocity.z = wz + g(0.0, self._gyro_sigma)
        msg.angular_velocity_covariance = [
            self._gyro_sigma**2, 0.0, 0.0,
            0.0, self._gyro_sigma**2, 0.0,
            0.0, 0.0, self._gyro_sigma**2,
        ]
        msg.linear_acceleration_covariance = [-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self._imu_pub.publish(msg)
