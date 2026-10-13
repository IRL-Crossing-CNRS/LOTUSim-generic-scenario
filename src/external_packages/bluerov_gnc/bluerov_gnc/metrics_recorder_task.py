"""Metrics recorder: writes one CSV time series and one JSON summary per agent.

Subscribes to the three GNC topics and to the per-thruster commands, and
records what each block produced, sample by sample, plus the aggregate
metrics used to compare runs:

    /<world>/<agent>/navigation      state estimate      (nav_msgs/Odometry)
    /<world>/<agent>/guidance        setpoint            (lotusim_msgs/GuidanceSetpoint)
    /<world>/<agent>/control         wrench demand       (geometry_msgs/WrenchStamped)
    /<world>/vessel_cmd_array        per-thruster thrust (lotusim_msgs/VesselCmdArray)

It is a pure observer: it publishes nothing and is not part of the control
loop, so adding or removing it cannot change a run's trajectory.

One sample is written per navigation message, pairing the latest guidance,
control and thruster values received. Position error is taken against
whatever the Guidance block asked for, so the same recorder serves station
keeping (error to the held point) and path following (cross-track error)
without knowing which mission is running.
"""

from __future__ import annotations

import csv
import json
import math
import os

from geometry_msgs.msg import WrenchStamped
from nav_msgs.msg import Odometry
from lotusim_msgs.msg import GuidanceSetpoint, VesselCmdArray

from lotusim_sdk.bt.status import Status
from lotusim_sdk.tasks.base import TaskAgent
from lotusim_sdk.control import enu_quat_to_ned_euler, enu_to_ned_position

from .allocation_task import BODY, PROPS
from .power import total_power

#: Time-series columns, in order.
COLUMNS = (
    ["t", "x", "y", "z", "phi", "theta", "psi", "u", "v", "w"]
    + [
        "target_x",
        "target_y",
        "desired_depth",
        "desired_heading",
        "desired_speed",
        "cross_track_error",
        "along_track_distance",
        "use_position_hold",
        "arrived",
    ]
    + ["pos_error", "depth_error", "heading_error"]
    + ["surge_N", "sway_N", "heave_N", "yaw_Nm"]
    + [f"T{i}" for i in PROPS]
    + ["thrust_norm_N", "power_W", "energy_Wh"]
)


def _wrap(angle: float) -> float:
    return (angle + math.pi) % (2 * math.pi) - math.pi


class BlueRovMetricsRecorderTask(TaskAgent):
    """Records the GNC topics to CSV, and aggregates run metrics.

    Params:
        output_dir     str    directory for the output files, created if
                              missing. Relative paths resolve against the
                              process working directory (the repository root
                              when launched by scenario_launch.sh).
                              Default "results/bluerov_current_experiment".
        run_name       str    subdirectory under output_dir, typically the
                              scenario name. Optional.
        settle_s       float  initial seconds excluded from the aggregate
                              metrics, so a run's start transient does not
                              dominate them (default 60). The CSV always
                              holds every sample regardless.
    """

    PERPETUAL = True

    def __init__(self, host, params=None, blackboard=None, id: str = "") -> None:
        super().__init__(host, params, blackboard, id)
        p = self.params
        out = str(p.get("output_dir", "results/bluerov_current_experiment"))
        run = str(p.get("run_name", "")).strip()
        self._dir = os.path.join(out, run) if run else out
        self._settle_s = float(p.get("settle_s", 60.0))

        self._nav_sub = None
        self._guidance_sub = None
        self._control_sub = None
        self._cmd_sub = None

        self._guidance = None
        self._wrench = None
        self._thrusts = {i: 0.0 for i in PROPS}

        self._csv_file = None
        self._csv = None
        self._rows = 0

        # Aggregates, accumulated only after settle_s.
        self._t0 = None
        self._t_prev = None
        self._power_prev = None
        self._energy_j = 0.0
        self._n = 0
        self._sum_sq_pos = 0.0
        self._max_pos = 0.0
        self._sum_sq_cross = 0.0
        self._max_cross = 0.0
        self._sum_sq_depth = 0.0
        self._sum_speed = 0.0
        self._sum_sq_thrust = 0.0
        self._saturated = 0

    # -- lifecycle --------------------------------------------------------
    def on_enter(self) -> None:
        world = self.host.world_name
        agent = self.host.agent_name
        os.makedirs(self._dir, exist_ok=True)
        path = os.path.join(self._dir, f"{agent}.csv")
        self._csv_file = open(path, "w", newline="")
        self._csv = csv.writer(self._csv_file)
        self._csv.writerow(COLUMNS)

        self._guidance_sub = self.host.create_subscription(
            GuidanceSetpoint, f"/{world}/{agent}/guidance", lambda m: setattr(self, "_guidance", m), 10
        )
        self._control_sub = self.host.create_subscription(
            WrenchStamped, f"/{world}/{agent}/control", lambda m: setattr(self, "_wrench", m), 10
        )
        self._cmd_sub = self.host.create_subscription(VesselCmdArray, f"/{world}/vessel_cmd_array", self._on_cmd, 10)
        # Subscribed last: every sample is written from a navigation message,
        # so the others are in place before the first one can arrive.
        self._nav_sub = self.host.create_subscription(Odometry, f"/{world}/{agent}/navigation", self._on_navigation, 10)

        self.host.get_logger().info(f"{type(self).__name__}: recording to {os.path.abspath(path)}")

    def on_exit(self, status) -> None:
        self._finalize()

    def update(self) -> Status:
        return Status.RUNNING

    def __del__(self):
        try:
            self._finalize()
        except Exception:
            pass

    # -- inputs -----------------------------------------------------------
    def _on_cmd(self, msg: VesselCmdArray) -> None:
        agent = self.host.agent_name
        for cmd in msg.cmds:
            if cmd.vessel_name != agent:
                continue
            try:
                parsed = json.loads(cmd.cmd_string)
            except (ValueError, TypeError):
                return
            for i in PROPS:
                key = f"{BODY}_prop_{i}(T)"
                if key in parsed:
                    self._thrusts[i] = float(parsed[key])
            return

    def _on_navigation(self, msg: Odometry) -> None:
        if self._csv is None:
            return
        t = self.host.current_pose_sim_time
        if t is None:
            return

        pose = msg.pose.pose
        x, y, z = enu_to_ned_position(pose.position.x, pose.position.y, pose.position.z)
        phi, theta, psi = enu_quat_to_ned_euler(
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w
        )
        vx, vy, vz = enu_to_ned_position(msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z)
        c, s = math.cos(psi), math.sin(psi)
        u = vx * c + vy * s
        v = -vx * s + vy * c
        w = vz

        g = self._guidance
        if g is None:
            return  # nothing to measure the error against

        # Position error: distance to the held point when Guidance is holding
        # one, cross-track error otherwise -- in both cases "how far from
        # where Guidance asked to be".
        cross = float(g.cross_track_error)
        if g.use_position_hold:
            pos_err = math.hypot(x - g.target_x, y - g.target_y)
        else:
            pos_err = abs(cross)
        depth_err = z - g.desired_depth
        heading_err = _wrap(g.desired_heading - psi)

        wr = self._wrench
        surge = wr.wrench.force.x if wr else 0.0
        sway = wr.wrench.force.y if wr else 0.0
        heave = wr.wrench.force.z if wr else 0.0
        yaw = wr.wrench.torque.z if wr else 0.0

        thrusts = [self._thrusts[i] for i in PROPS]
        thrust_norm = math.sqrt(sum(tt * tt for tt in thrusts))
        power = total_power(thrusts)

        # Energy: trapezoidal integration on simulated time, over the whole
        # run (not only the post-settle window) since it is a total, not a
        # steady-state statistic.
        if self._t_prev is not None and t > self._t_prev:
            self._energy_j += 0.5 * (power + self._power_prev) * (t - self._t_prev)
        self._t_prev, self._power_prev = t, power
        if self._t0 is None:
            self._t0 = t

        self._csv.writerow(
            [
                f"{t:.3f}",
                f"{x:.4f}",
                f"{y:.4f}",
                f"{z:.4f}",
                f"{phi:.6f}",
                f"{theta:.6f}",
                f"{psi:.6f}",
                f"{u:.4f}",
                f"{v:.4f}",
                f"{w:.4f}",
                f"{g.target_x:.4f}",
                f"{g.target_y:.4f}",
                f"{g.desired_depth:.4f}",
                f"{g.desired_heading:.6f}",
                f"{g.desired_speed:.4f}",
                f"{cross:.4f}",
                f"{g.along_track_distance:.4f}",
                int(bool(g.use_position_hold)),
                int(bool(g.arrived)),
                f"{pos_err:.4f}",
                f"{depth_err:.4f}",
                f"{heading_err:.6f}",
                f"{surge:.3f}",
                f"{sway:.3f}",
                f"{heave:.3f}",
                f"{yaw:.3f}",
                *[f"{tt:.3f}" for tt in thrusts],
                f"{thrust_norm:.3f}",
                f"{power:.2f}",
                f"{self._energy_j / 3600.0:.5f}",
            ]
        )
        self._rows += 1

        if t - self._t0 < self._settle_s:
            return
        self._n += 1
        self._sum_sq_pos += pos_err * pos_err
        self._max_pos = max(self._max_pos, pos_err)
        self._sum_sq_cross += cross * cross
        self._max_cross = max(self._max_cross, abs(cross))
        self._sum_sq_depth += depth_err * depth_err
        self._sum_speed += math.sqrt(u * u + v * v + w * w)
        self._sum_sq_thrust += sum(tt * tt for tt in thrusts)
        if any(abs(tt) >= 49.999 for tt in thrusts):
            self._saturated += 1

    # -- output -----------------------------------------------------------
    def _finalize(self) -> None:
        if self._csv_file is None:
            return
        agent = self.host.agent_name
        n = self._n
        summary = {
            "agent": agent,
            "samples_total": self._rows,
            "samples_after_settle": n,
            "settle_s": self._settle_s,
            "duration_s": (
                round((self._t_prev - self._t0), 3) if self._t0 is not None and self._t_prev is not None else 0.0
            ),
            "rms_pos_error_m": round(math.sqrt(self._sum_sq_pos / n), 4) if n else None,
            "max_pos_error_m": round(self._max_pos, 4) if n else None,
            "rms_cross_track_m": round(math.sqrt(self._sum_sq_cross / n), 4) if n else None,
            "max_cross_track_m": round(self._max_cross, 4) if n else None,
            "rms_depth_error_m": round(math.sqrt(self._sum_sq_depth / n), 4) if n else None,
            "mean_speed_ms": round(self._sum_speed / n, 4) if n else None,
            # Control effort: RMS of the per-thruster commands, i.e. over all
            # samples AND all six thrusters.
            "rms_control_effort_N": round(math.sqrt(self._sum_sq_thrust / (n * len(PROPS))), 4) if n else None,
            "saturated_fraction": round(self._saturated / n, 4) if n else None,
            "energy_wh": round(self._energy_j / 3600.0, 5),
        }
        path = os.path.join(self._dir, f"{agent}_summary.json")
        with open(path, "w") as f:
            json.dump(summary, f, indent=2)
            f.write("\n")

        self._csv_file.close()
        self._csv_file = None
        self._csv = None
        self.host.get_logger().info(
            f"{type(self).__name__}: {self._rows} samples, "
            f"energy {summary['energy_wh']} Wh, summary -> {os.path.abspath(path)}"
        )
