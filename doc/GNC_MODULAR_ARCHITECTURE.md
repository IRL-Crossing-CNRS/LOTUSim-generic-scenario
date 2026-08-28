# Modular GNC architecture — Navigation / Guidance / Control / Allocation

A pipeline of four blocks plus an optional path-planning block, each a
distinct ROS2 node, connected only through topics with a fixed message
schema. A user who wants to plug their own algorithm for one block (their own
EKF, their own MPC, ...) replaces the node that publishes/subscribes that
block's topics; no other block's code is touched, because no block depends on
another block's implementation — only on its message schema.

This is a specification; section 6 shows `bluerov_gnc` implementing it.

## 1. Block diagram

```
 sensors (IMU, DVL, pressure, GPS, or simulator ground truth)
        │
        ▼
 ┌─────────────┐   /<world>/<agent>/navigation   nav_msgs/Odometry
 │  NAVIGATION │ ──────────────────────────────────────────────────►
 └─────────────┘   state estimate x̂: pose + twist + covariance
        │
        │ (consumed by both Guidance and Control)
        │
 ┌─────────────┐   /<world>/<agent>/path          nav_msgs/Path
 │    PATH     │ ──────────────────────────────────────────────────►
 │  PLANNING   │   optional: a sequence of waypoints
 └─────────────┘
        │
        ▼
 ┌─────────────┐   /<world>/<agent>/guidance      lotusim_msgs/GuidanceSetpoint
 │  GUIDANCE   │ ──────────────────────────────────────────────────►
 └─────────────┘   desired heading, depth, speed, cross-track error
        │
        ▼
 ┌─────────────┐   /<world>/<agent>/control       geometry_msgs/WrenchStamped
 │   CONTROL   │ ──────────────────────────────────────────────────►
 └─────────────┘   body-frame generalized force/torque demand
        │
        ▼
 ┌─────────────┐   /<world>/vessel_cmd_array      lotusim_msgs/VesselCmdArray
 │ ALLOCATION  │ ──────────────────────────────────────────────────►
 └─────────────┘   per-actuator command (thrust, rpm, control surface, ...)
        │
        ▼
   simulator (xdyn / Gazebo KinematicInterface) or a real thruster driver
```

Navigation reads sensors and publishes a state estimate. Guidance and Control
both read that estimate directly; Guidance additionally reads a path if one
is published. Control never reads sensors or the path — only the state
estimate and the guidance setpoint. Allocation never reads anything upstream
of Control — only the wrench demand. This one-directional, topic-only
dependency is what makes each block independently replaceable.

## 2. Topics and ownership

| topic | message | published by | consumed by |
|---|---|---|---|
| `/<world>/poses` | `lotusim_msgs/VesselPositionArray` | simulator | Navigation |
| `/<world>/<agent>/navigation` | `nav_msgs/Odometry` | Navigation | Guidance, Control |
| `/<world>/<agent>/path` | `nav_msgs/Path` | Path Planning | Guidance |
| `/<world>/<agent>/guidance` | `lotusim_msgs/GuidanceSetpoint` | Guidance | Control |
| `/<world>/<agent>/control` | `geometry_msgs/WrenchStamped` | Control | Allocation |
| `/<world>/vessel_cmd_array` | `lotusim_msgs/VesselCmdArray` | Allocation | simulator or real thrusters |

`/<world>/poses` and `/<world>/vessel_cmd_array` are existing, shared topics
(all vessels in a world publish/subscribe on the same name, keyed by
`vessel_name` inside the message). The four per-agent topics are namespaced
`/<world>/<agent>/...`, the existing convention used by
`check_battery_state.py` and `fault_inspection.py`, so N agents in the same
world do not collide.

## 3. Message schemas

**`nav_msgs/Odometry`** (standard ROS2 message, already available) — pose,
twist and their covariances, in the world frame. Chosen over a custom message
so that a third-party estimator (e.g. `robot_localization`'s EKF/UKF nodes)
can publish directly to `/<world>/<agent>/navigation` with no adaptation
layer.

**`lotusim_msgs/GuidanceSetpoint`**, defined in
`LOTUSim/interfaces/lotusim_msgs/msg/GuidanceSetpoint.msg` —

```
std_msgs/Header header
bool    use_position_hold      # true: Control drives directly toward (target_x, target_y)
float64 target_x               # m, valid only if use_position_hold
float64 target_y               # m, valid only if use_position_hold
float64 desired_heading        # rad, positive clockwise from above
float64 desired_depth          # m, positive down
float64 desired_speed          # m/s, surge -- valid only if not use_position_hold
float64 cross_track_error      # m, signed; 0 if use_position_hold
float64 along_track_distance   # m, from the start of the current leg; 0 if use_position_hold
bool    arrived
```

`use_position_hold` lets Control stay a single, mode-agnostic node: station
keeping and the end-of-segment hold both set it true and give a direct (x, y)
target, while path-following guidance laws (LOS, pure-pursuit) set it false and
drive on heading + speed instead. Sufficient for LOS, pure-pursuit and
waypoint-switching guidance laws, and generic enough to serve both a surface
vehicle (`desired_depth` unused) and an underwater one.

**`geometry_msgs/WrenchStamped`** (standard ROS2 message) — `wrench.force`
(surge, sway, heave, N) and `wrench.torque` (roll, pitch, yaw, N·m), in the
body frame. This is the generalized force/moment demand Control hands to
Allocation, regardless of how many actuators the vehicle has or how they are
arranged — that arrangement is Allocation's problem alone.

**`lotusim_msgs/VesselCmdArray`** (existing) — one `VesselCmd` per actuator,
`cmd_string` a JSON object of named commands (thrust in N, rpm, control
surface angle, ...). This is already the message the simulator's
`PhysicsInterfacePlugin` and `KinematicInterface` consume.

## 4. Algorithms per block

| block | example algorithms |
|---|---|
| Navigation | ground-truth passthrough (simulation only); complementary filter; EKF/UKF (e.g. `robot_localization`); particle filter (acoustic-only underwater positioning) |
| Path planning | static waypoint list; A*/theta* on a costmap; Dubins path; RRT*/PRM |
| Guidance | line-of-sight (straight segment); pure-pursuit (waypoint following); Dubins-path follower; potential-field or velocity-obstacle guidance (multi-agent) |
| Control | PID; LQR; sliding-mode; backstepping; MPC |
| Allocation | fixed pseudo-inverse matrix (decoupled axes); QP-based allocation with saturation and control-effort weighting; direct per-DOF mapping for a minimally-actuated vehicle |

A user plugging in, say, an MPC controller writes one ROS2 node that
subscribes `/<world>/<agent>/navigation` and `/<world>/<agent>/guidance` and
publishes `/<world>/<agent>/control` — Guidance and Allocation are unaware
anything changed.

## 5. QoS

State/setpoint/command topics (`navigation`, `guidance`, `control`,
`vessel_cmd_array`) are periodic, latest-value-matters streams: `VOLATILE`
durability, `RELIABLE` reliability, depth matched to the control rate (a late
joiner does not need history, only the next sample). `path` is
`TRANSIENT_LOCAL` durability so a late-joining Guidance node still gets the
most recently published path without waiting for the next planning cycle —
the same reasoning already applied to `renderer_cmd` (`KeepAll` so late
subscribers get every spawn).

## 6. Example: `bluerov_gnc`

`bluerov_gnc` implements this decomposition as four ROS2 tasks, one per
block, connected only through the topics in §2 — no block calls another's
code directly. Path planning is not used: the two mission endpoints come
directly from scenario JSON parameters.

| block | example implemented | file |
|---|---|---|
| Navigation | ground-truth passthrough: republishes `host.current_pose`/`current_twist`, the simulator's ground truth, as `nav_msgs/Odometry`. No sensor model, no estimation error | `navigation_task.py`, `BlueRovNavigationTask` |
| Guidance | two interchangeable examples: line-of-sight (`BlueRovLOSGuidanceTask`) and pure-pursuit (`BlueRovPurePursuitGuidanceTask`), both wrapping the matching class in `lotusim_sdk.control.guidance`; a third, `BlueRovHoldGuidanceTask`, holds a fixed point for station keeping | `guidance_tasks.py` |
| Control | PID: `DepthHoldPID`/`HeadingHoldPID`/`PositionHoldPID`/`SurgeSpeedPID` from `lotusim_sdk.control.pid`, selected per message via `use_position_hold` rather than being written per-mission | `control_task.py`, `BlueRovControlTask` |
| Allocation | a fixed, analytically-inverted allocation matrix for this vehicle's symmetric thruster layout (not a general pseudo-inverse or a QP-based allocation) — `ThrusterAllocator` | `thruster_allocation.py`, wired by `allocation_task.py`'s `BlueRovAllocationTask` |

A fifth task, `BlueRovMetricsRecorderTask` (`metrics_recorder_task.py`), is
**not** part of the pipeline: it subscribes to all four topics above and
writes a CSV time series plus a JSON summary, publishing nothing. Being a
pure observer, adding or removing it cannot change a run — and it works for
any implementation of any block, since it reads only the message schemas.

Swapping the transect's guidance law from line-of-sight to pure-pursuit is a
one-line change in the scenario JSON (`bluerov_current_experiment/transect_ekman_pure_pursuit.json`
against `bluerov_current_experiment/transect_ekman.json`) — Navigation, Control and Allocation are
unchanged. This is the concrete demonstration of the swap principle in §1: a
different guidance node, same topics, same message schema, no other code
touched.
