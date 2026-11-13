# Navigation, guidance and control tasks

The tasks of the navigation, guidance, control and allocation pipeline. A
vehicle runs one task of each block, each as its own tree in `missions`; they
never finish. The blocks talk only through their topics, so any one of them
can be swapped without touching the others. How the pipeline is built is in
[GNC_MODULAR_ARCHITECTURE.md](../GNC_MODULAR_ARCHITECTURE.md).

| Block | Publishes | Tasks |
|---|---|---|
| Navigation | `/<world>/<agent>/navigation` | `navigation` |
| Guidance | `/<world>/<agent>/guidance` | `guidance_hold`, `guidance_los`, `guidance_pure_pursuit`, `guidance_los_polyline`, `guidance_pure_pursuit_polyline` |
| Control | `/<world>/<agent>/control` | `control`, and per vehicle `bluerov_control`, `lrauv_control`, `wamv_control`, `dtmb_hull_control` |
| Allocation | `/<world>/vessel_cmd_array` | `kinematic_allocation`, `static_command_allocation`, and per vehicle `bluerov_allocation`, `lrauv_allocation`, `wamv_allocation` |

Frames: positions are North, East, Down (NED): depths positive down,
headings clockwise from North, as in xdyn. For an aerial vehicle an altitude of 40 m is a depth of −40.

## Which tasks for which vehicle

| Vehicle | Control | Allocation, Kinematic | Allocation, xdyn |
|---|---|---|---|
| BlueROV2 | `bluerov_control` | — | `bluerov_allocation` |
| LRAUV | `lrauv_control` | `kinematic_allocation` | `lrauv_allocation` |
| WAM-V | `wamv_control` | `kinematic_allocation` | `wamv_allocation` |
| X500 | `control` | `kinematic_allocation` | — |
| Drifting under xdyn | — | — | none, or `static_command_allocation` for a vehicle whose model declares actuators |

Examples of each: `current_examples/` (BlueROV2) and `multi_vehicle_examples/`.

## Navigation

### `navigation`

Publishes the vehicle's state (`nav_msgs/Odometry`) from the simulator's
true pose. It is not an estimator: no noise, no filter. The message follows REP
105: the pose in the world frame (ENU), the twist in the body frame (FLU,
`child_frame_id` = the agent), so a ROS 2 state estimator can replace this task
on the same topic.

| Param | Type | Default | Meaning |
|---|---|---|---|
| `control_rate_hz` | float | `20` | Publish rate |

`bluerov_navigation` is the same task under another name.

### `gnss_imu`

Noisy measurements for a state estimator that replaces `navigation`, made from
the true pose with seeded Gaussian noise: a horizontal position fix on
`/<world>/<agent>/gnss` (`geometry_msgs/PoseWithCovarianceStamped`) and an
orientation and angular velocity on `/<world>/<agent>/imu` (`sensor_msgs/Imu`;
linear acceleration is not modelled). The covariances are the noise variances.

| Param | Type | Default | Meaning |
|---|---|---|---|
| `gnss_rate_hz` | float | `5` | Position fix rate |
| `gnss_sigma_m` | float | `1.0` | Horizontal noise per axis (m) |
| `imu_rate_hz` | float | `50` | IMU rate |
| `heading_sigma_deg` | float | `1.0` | Heading noise |
| `tilt_sigma_deg` | float | `0.5` | Roll and pitch noise |
| `gyro_sigma_rad_s` | float | `0.005` | Angular velocity noise |

## Guidance

All guidance tasks publish a `lotusim_msgs/GuidanceSetpoint`: target depth,
heading and speed, or a point to hold.

### `guidance_hold`

Holds a point, a depth and a heading.

| Param | Type | Default | Meaning |
|---|---|---|---|
| `hold_origin` | bool | `true` | Hold the position of the first pose received; `false` holds `x_setpoint_m`, `y_setpoint_m` |
| `x_setpoint_m`, `y_setpoint_m` | float (m) | `0`, `0` | Point to hold when `hold_origin` is `false`, North and East in the world's NED frame |
| `z_setpoint_m` | float (m) | `3` | Depth, positive down |
| `psi_setpoint_deg` | float (deg) | `0` | Heading |

### `guidance_los`, `guidance_pure_pursuit`

Follow one straight segment from the first pose received, then hold its end.
`guidance_los` uses line-of-sight guidance, `guidance_pure_pursuit` pure
pursuit; their parameters are the same.

| Param | Type | Default | Meaning |
|---|---|---|---|
| `x_end_m`, `y_end_m` | float (m) | `200`, `0` | End of the segment, North and East of the start |
| `z_start_m`, `z_end_m` | float (m) | `3`, `55` | Depth at the start and at the end |
| `u_setpoint_ms` | float (m/s) | `0.5` | Speed along the segment |
| `lookahead_m` | float (m) | `5` | Lookahead distance: larger is smoother, smaller follows the line more tightly |

### `guidance_los_polyline`, `guidance_pure_pursuit_polyline`

Follow a list of waypoints, one segment after another, then hold the last
one. A curved path is written as a polyline.

| Param | Type | Default | Meaning |
|---|---|---|---|
| `waypoints` | list, required | — | `[[dx, dy, z], ...]`: `dx` North and `dy` East of the start, in metres, `z` the depth at that waypoint |
| `z_start_m` | float (m) | `3` | Depth at the start |
| `u_setpoint_ms` | float (m/s) | `0.5` | Speed along the path |
| `lookahead_m` | float (m) | `5` | Lookahead distance |

The `bluerov_guidance_*` tasks are the same tasks under other names.

## Control

### `control`

Four PID loops (depth, heading, position hold, surge speed) turn navigation
and guidance into a force and moment demand (`geometry_msgs/WrenchStamped`).
The loop steps once per new navigation message, so a rate above the world's
step does not add oscillation.

| Param | Type | Default | Meaning |
|---|---|---|---|
| `control_rate_hz` | float | `20` | Loop rate, limited in practice by the navigation rate |
| `kp_z`, `ki_z`, `kd_z` | float | `-120`, `-10`, `-60` | Depth gains |
| `kp_psi`, `ki_psi`, `kd_psi` | float | `4`, `0.1`, `2` | Heading gains |
| `kp_xy`, `ki_xy`, `kd_xy` | float | `60`, `4`, `40` | Position-hold gains |
| `kp_u`, `ki_u`, `kd_u` | float | `120`, `20`, `0` | Surge-speed gains |
| `feedforward` | object | none | Current feedforward, see below |
| `ff_gain` | float | `1.0` | Scales the feedforward |
| `xu`, `xuu`, `yv`, `yvv` | float | `0`, or the vehicle's | Linear and quadratic damping in surge and sway, used to size the feedforward |

The outputs are limited to 6 N·m in yaw, 141 N in surge and position hold,
100 N in heave: values sized for the BlueROV2.

**Current feedforward.** With `feedforward`, the controller predicts the
current at its own depth and pushes against the drag it will cause. The
model the controller uses is independent of the current the scenario
applies: to compare models, keep the applied current fixed and change only
this block.

| `model` | Other keys |
|---|---|
| `"none"` (default) | — |
| `"uniform"` (also `"gauss"`, `"gauss_markov"`) | `mean_x`, `mean_y` (m/s, default 0) |
| `"ekman"` | `current_velocity_ms` (0), `current_orientation_deg` (0), `top_layer_thickness_m` (10), `bottom_layer_thickness_m` (20), `U10_ms` (0), `seabed_depth_m` (65), `latitude_deg` (47), `wind_orientation_deg` (20) |

### Per-vehicle control tasks

| Task | Differs from `control` by |
|---|---|
| `bluerov_control` | The BlueROV2's damping as feedforward defaults: `xu` 13.7, `xuu` 141, `yv` 0, `yvv` 217 |
| `lrauv_control`, `dtmb_hull_control` | Nothing yet: no damping identified, so no feedforward unless the scenario gives `xu`… |
| `wamv_control` | Three more parameters: `n_max_nm` (yaw limit, default 6), `f_max_n` (surge limit, default 141), `hold_radius_m` (holding a point without sway: outside this radius the boat points at the point and drives back; default off). See [the WAM-V model](../WAMV_MODEL.md) |

## Allocation

### `kinematic_allocation`

For a vehicle without hydrodynamics: turns the demand into a commanded surge
speed, yaw rate and vertical speed, which the simulator follows exactly.

| Param | Type | Default | Meaning |
|---|---|---|---|
| `u_gain`, `w_gain`, `vz_gain` | float | `0.05` each | Surge speed per newton, yaw rate per N·m, vertical speed per newton |
| `u_max_ms`, `w_max_rads`, `vz_max_ms` | float | `2.0`, `1.0`, `1.0` | Limits of each |

### `static_command_allocation`

Sends one fixed command forever. Use it to let a vehicle drift under xdyn
when its model declares actuators, which need a command every step.

| Param | Type | Default | Meaning |
|---|---|---|---|
| `commands` | object, required | — | The command keys of the vehicle's xdyn model and their values, e.g. `{"propeller(rpm)": 15.0, "propeller(P/D)": 0.88}` for the LRAUV |

### `bluerov_allocation`

Splits the demand over the six thrusters of the BlueROV2's xdyn model (four
horizontal, two vertical). Roll and pitch are not commanded.

| Param | Type | Default | Meaning |
|---|---|---|---|
| `t_max_n` | float (N) | `50` | Thrust limit per thruster |

### `lrauv_allocation`

Turns the surge demand into the LRAUV's propeller speed. There is no fin
control: the LRAUV holds its depth but cannot turn.

| Param | Type | Default | Meaning |
|---|---|---|---|
| `rpm_gain` | float | `1.0` | rpm per newton of surge demand |
| `rpm_max` | float | `300` | Propeller speed limit |
| `rpm_min` | float | `30` | Lowest commanded speed: the propeller model cannot take 0 rpm |
| `j_safe` | float | `1.0` | Highest advance ratio allowed, which sets a speed-dependent rpm floor |
| `rpm_slew_max` | float | `5` | Largest rpm change per step |
| `pitch_ratio` | float | `0.88` | Propeller pitch over diameter |

### `wamv_allocation`

Splits surge and yaw between the WAM-V's two stern engines. When the demand
exceeds the limit, steering is kept and surge reduced.

| Param | Type | Default | Meaning |
|---|---|---|---|
| `t_max_n` | float (N) | `250` | Thrust limit per engine, forward and reverse |

## Recording the pipeline

### `bluerov_metrics_recorder`

Records the four topics of the pipeline to a CSV time series and writes a
JSON summary of the run. It only listens, so adding it does not change the
run.

| Param | Type | Default | Meaning |
|---|---|---|---|
| `output_dir` | string | `"results/bluerov_current_experiment"` | Output folder, relative to the folder the launcher runs from |
| `run_name` | string | `""` | Subfolder of `output_dir`, usually the scenario's name |
| `settle_s` | float (s) | `60` | Start of the run left out of the summary, so the start-up transient does not dominate it |
