# Task reference

Every task a mission can run, by its name in `"task"`. Each page gives all
the parameters of its tasks, their defaults, and what the task publishes.

| Task | Does | Finishes | Page |
|---|---|---|---|
| `waypoint_follower` | Follows lat/lon waypoints with its own kinematic guidance | when the last waypoint is reached, unless `loop` | [Patrol tasks](tasks_patrol.md) |
| `waypoint_follower_avoidance` | The same, avoiding obstacles seen by a sonar | idem | [Patrol tasks](tasks_patrol.md) |
| `kinematic_anchor` | Keeps a vehicle without thrusters in the Kinematic physics, so it drifts with an `OceanCurrent` | never | [Patrol tasks](tasks_patrol.md) |
| `navigation` | Publishes the vehicle's state | never | [GNC tasks](tasks_gnc.md) |
| `guidance_hold` | Holds a point, a depth, a heading | never | [GNC tasks](tasks_gnc.md) |
| `guidance_los`, `guidance_pure_pursuit` | Follows a straight segment | never | [GNC tasks](tasks_gnc.md) |
| `guidance_los_polyline`, `guidance_pure_pursuit_polyline` | Follows a list of waypoints | never | [GNC tasks](tasks_gnc.md) |
| `control`, `bluerov_control`, `lrauv_control`, `wamv_control`, `dtmb_hull_control` | PID control, with an optional current feedforward | never | [GNC tasks](tasks_gnc.md) |
| `kinematic_allocation`, `static_command_allocation`, `bluerov_allocation`, `lrauv_allocation`, `wamv_allocation` | Turns the control demand into the vehicle's commands | never | [GNC tasks](tasks_gnc.md) |
| `bluerov_metrics_recorder` | Records the GNC topics and a summary of the run | never | [GNC tasks](tasks_gnc.md) |
| `fault_inspection` | Detects corrosion and cracks on the camera images | never | [Inspection and battery tasks](tasks_inspection.md) |
| `crack_detector` | Detects cracks with a YOLO detector | never | [Inspection and battery tasks](tasks_inspection.md) |
| `check_battery_state` | Lights the status light while the battery is above a threshold | never | [Inspection and battery tasks](tasks_inspection.md) |
| `blink_light` | Blinks the status light | never | [Inspection and battery tasks](tasks_inspection.md) |
| `px4_offboard_patrol` | Arms a PX4 drone, takes off and flies a waypoint list | when the list is done, unless `loop` | [PX4 drones](../PX4_DRONES.md#4-flying-it-manual-or-offboard) |
| `set_wind` | Sets the ambient wind | at once | [Wait and wind tasks](tasks_utility.md) |
| `wait` | Waits a given time | after `duration_s` | [Wait and wind tasks](tasks_utility.md) |

Tasks that never finish belong in their own tree or in a `parallel`, see
[Missions](missions.md#how-a-mission-runs).
