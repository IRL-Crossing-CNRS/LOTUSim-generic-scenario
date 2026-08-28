# Modular GNC architecture — Navigation / Guidance / Control / Allocation

A pipeline of four blocks plus an optional path-planning block, each a
distinct ROS2 node, connected only through topics with a fixed message
schema. A user who wants to plug their own algorithm for one block (their own
EKF, their own MPC, ...) replaces the node that publishes/subscribes that
block's topics; no other block's code is touched, because no block depends on
another block's implementation — only on its message schema.

Sections 1–5 are the specification. Section 6 lists what is implemented
today, section 7 what is not.

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
 ┌─────────────┐   /<world>/<agent>/waypoints     lotusim_msgs/SetWaypoints
 │    PATH     │ ──────────────────────────────────────────────────►
 │  PLANNING   │   optional, not wired yet (see §7): a waypoint sequence
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
 └─────────────┘   per-actuator command (thrust, rpm, {u,w,vz}, ...)
        │
        ▼
   simulator (xdyn / Gazebo KinematicInterface) or a real thruster driver
```

Navigation reads sensors and publishes a state estimate. Guidance and Control
both read that estimate directly; Guidance additionally takes a path if one
is supplied. Control never reads sensors or the path — only the state
estimate and the guidance setpoint. Allocation never reads anything upstream
of Control — only the wrench demand. This one-directional, topic-only
dependency is what makes each block independently replaceable.

## 2. Topics and ownership

| topic | message | published by | consumed by |
|---|---|---|---|
| `/<world>/poses` | `lotusim_msgs/VesselPositionArray` | simulator | Navigation |
| `/<world>/<agent>/navigation` | `nav_msgs/Odometry` | Navigation | Guidance, Control |
| `/<world>/<agent>/waypoints` (service) | `lotusim_msgs/SetWaypoints` | Path Planning | Guidance (not wired, §7) |
| `/<world>/<agent>/guidance` | `lotusim_msgs/GuidanceSetpoint` | Guidance | Control |
| `/<world>/<agent>/control` | `geometry_msgs/WrenchStamped` | Control | Allocation |
| `/<world>/vessel_cmd_array` | `lotusim_msgs/VesselCmdArray` | Allocation | simulator or real thrusters |

`/<world>/poses` and `/<world>/vessel_cmd_array` are existing, shared topics
(all vessels in a world publish/subscribe on the same name, keyed by
`vessel_name` inside the message). The per-agent topics are namespaced
`/<world>/<agent>/...`, the existing convention used by
`check_battery_state.py` and `fault_inspection.py`, so N agents in the same
world do not collide.

Today a mission's waypoints reach Guidance as static scenario-JSON params
(`waypoints`, or the endpoint parameters of a single-segment law), read once
when the behaviour tree is built. `SetWaypoints` is the seam intended to
replace that with a live path source; §7 states exactly what is missing.

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
waypoint-switching guidance laws, and generic enough to serve a surface
vehicle (`desired_depth` unused), an underwater one, and an aerial one
(`desired_depth` negative, i.e. an altitude above the surface).

**`geometry_msgs/WrenchStamped`** (standard ROS2 message) — `wrench.force`
(surge, sway, heave, N) and `wrench.torque` (roll, pitch, yaw, N·m), in the
body frame. This is the generalized force/moment demand Control hands to
Allocation, regardless of how many actuators the vehicle has or how they are
arranged — that arrangement is Allocation's problem alone.

**`lotusim_msgs/VesselCmdArray`** (existing) — one `VesselCmd` per commanded
entity, `cmd_string` a JSON object of named commands. The schema inside that
JSON is the consumer's, not the message's: named thruster/actuator values for
xdyn (`{"propeller(rpm)": ...}`, thrust in N for the BlueROV model), and
`{"u", "w", "vz"}` for `KinematicInterface`. This is already the message the
simulator's `PhysicsInterfacePlugin` and `KinematicInterface` consume.

**`lotusim_msgs/SetWaypoints`** (service) — `geographic_msgs/GeoPoint[] path`
plus a `loop` flag, answering `bool success`.

## 4. Algorithms per block

| block | example algorithms |
|---|---|
| Navigation | ground-truth passthrough (simulation only); complementary filter; EKF/UKF (e.g. `robot_localization`); particle filter (acoustic-only underwater positioning) |
| Path planning | static waypoint list; A*/theta* on a costmap; Dubins path; RRT*/PRM |
| Guidance | line-of-sight (straight segment or polyline); pure-pursuit; station keeping; Dubins-path follower; potential-field or velocity-obstacle guidance (multi-agent) |
| Control | PID (optionally with a model-based current feedforward, §6); LQR; sliding-mode; backstepping; MPC |
| Allocation | fixed pseudo-inverse matrix (decoupled axes); QP-based allocation with saturation and control-effort weighting; direct per-DOF mapping for a minimally-actuated vehicle; gain mapping to a kinematic velocity command |

A user plugging in, say, an MPC controller writes one ROS2 node that
subscribes `/<world>/<agent>/navigation` and `/<world>/<agent>/guidance` and
publishes `/<world>/<agent>/control` — Guidance and Allocation are unaware
anything changed.

## 5. QoS

State/setpoint/command topics (`navigation`, `guidance`, `control`,
`vessel_cmd_array`) are periodic, latest-value-matters streams, and are
created with the default reliable QoS at depth 10: a late joiner does not
need history, only the next sample. Waypoints are a service call, not a
topic, so a late-joining Guidance node is answered on request rather than
needing `TRANSIENT_LOCAL` durability.

## 6. What is implemented

### Generic, vehicle-agnostic tasks (`lotusim_sdk.tasks`)

These are the default implementation of each block. Any vehicle class can
register them directly; a vehicle only writes its own task when the block is
genuinely vehicle-specific (Allocation, and Control when vehicle damping
coefficients are known).

| block | task registry name | class | file |
|---|---|---|---|
| Navigation | `navigation` | `NavigationTask` | `lotusim_sdk/tasks/navigation.py` |
| Guidance | `guidance_hold` | `HoldGuidanceTask` | `lotusim_sdk/tasks/guidance.py` |
| Guidance | `guidance_los` | `LOSGuidanceTask` | idem |
| Guidance | `guidance_pure_pursuit` | `PurePursuitGuidanceTask` | idem |
| Guidance | `guidance_los_polyline` | `LOSPolylineGuidanceTask` | idem |
| Guidance | `guidance_pure_pursuit_polyline` | `PurePursuitPolylineGuidanceTask` | idem |
| Control | `control` | `ControlTask` | `lotusim_sdk/tasks/control.py` |
| Allocation | `kinematic_allocation` | `KinematicAllocationTask` | `lotusim_sdk/tasks/kinematic_allocation.py` |
| Allocation | `static_command_allocation` | `StaticCommandAllocationTask` | `lotusim_sdk/tasks/static_command_allocation.py` |

Navigation is a ground-truth passthrough: it republishes the simulator's
pose/twist as `nav_msgs/Odometry`, and falls back to a fixed-gain exponential
filter on a two-point position difference when the backend supplies no twist.
It is not an estimator — it is the seam where one plugs in.

The two polyline guidance laws take an explicit waypoint list, so a path whose
heading or depth varies along the way (a survey lawnmower, a vertical
sinusoid) is expressed by sampling it into segments; the single-segment laws
cannot represent one.

Control is four decoupled, clamped PID loops with anti-windup (depth, heading,
position hold, surge speed) from `lotusim_sdk.control.pid`, selected per
message via `use_position_hold` rather than per mission. It optionally adds a
**model-based current feedforward** on the horizontal axes
(`lotusim_sdk/control/current_feedforward.py`, `feedforward` params block:
`{"model": "none"|"uniform"|"ekman", ...}`). The feedforward is the channel
through which a current model reaches the *controller* rather than only the
environment: the controller queries a current model at its own depth and
pre-compensates the drag that current induces. The model the controller
believes in is independent of the current actually injected into the
simulation, which is what makes model-vs-model comparisons controlled.

`kinematic_allocation` maps the wrench to `{u, w, vz}` for
`KinematicInterface` through plain clamped gains — there is no thruster
geometry to invert, because Kinematic has no physics. It exists so a
Kinematic-backed vehicle sits downstream of the same generic Control as an
xdyn-backed one. `static_command_allocation` publishes one fixed resting
command forever, for a drift-only demo of a vehicle whose xdyn YAML declares
actuator command keys (xdyn stalls until it receives a complete command set).

### Vehicle-specific packages

| package | provides | notes |
|---|---|---|
| `bluerov_gnc` | `bluerov_allocation` (`ThrusterAllocator`), `bluerov_control`, `bluerov_metrics_recorder` | Navigation and the guidance tasks are re-exported aliases of the generic ones, kept for the existing `bluerov_*` registry names. The allocator analytically inverts the 6 modelled thrusters of the 8 (4 vectored + 2 vertical), so roll and pitch are not commandable |
| `lrauv_gnc` | `LrauvControlTask`, `LrauvAllocationTask` | Allocation drives a single propeller in surge; no yaw/fin actuation, so the vehicle goes straight once launched |
| `wamv_gnc` | `WamvControlTask` | Kinematic only — `wamv.yaml` has no thruster model to allocate against |
| `dtmb_hull_gnc` | `DtmbHullControlTask` | Kinematic only |

`BlueRovMetricsRecorderTask` is **not** part of the pipeline: it subscribes to
all four topics and writes a CSV time series plus a JSON summary, publishing
nothing. Being a pure observer, adding or removing it cannot change a run —
and it works for any implementation of any block, since it reads only the
message schemas.

### Scenarios

`src/simulation_run/config/current_examples/` runs the pipeline on a BlueROV2
across the four ocean-current models;
`src/simulation_run/config/multi_vehicle_examples/` runs it on every other
vehicle domain. Both directories' READMEs, and the capability table in
`src/simulation_run/config/README.md`, state per vehicle which backend
(Kinematic or xdyn) is controllable and which is drift-only.

Swapping a guidance law is a one-line change in the scenario JSON: the `task`
field of the guidance mission selects the node, and Navigation, Control and
Allocation are unchanged.

## 7. What is not implemented

- **A live path source.** The `SetWaypoints` service type exists and a client
  helper exists (`simulation_run/dynamic_spawn/set_waypoints.py`, which talks
  to the Gazebo waypoint plugin, not to Guidance), but no Guidance task
  serves that service, and nothing calls it into Guidance. Missions are
  therefore static: changing one means editing the scenario JSON and
  restarting. Closing this gap means (a) serving the service in
  `_GuidanceTaskBase` and (b) adding a client — a path-planning node, an RViz
  panel, or a mission task.
- **Aerial rotor-mixer allocation.** X500 in the generic pipeline runs
  through `kinematic_allocation`. A real mixer publishing
  `gz.msgs.Actuators` on `command/motor_speed`, consumed by the
  `gz-sim-multicopter-motor-model-system` plugin already present in every
  X500 SDF, is work in progress on a separate branch and its trajectory is
  not yet stable. PX4 SITL remains the separate, non-GNC path for a
  natively-flown drone.
- **xdyn allocation for WAMV, dtmb_hull and the hull-only surface vessels.**
  Their xdyn YAMLs declare no thruster model, so there is nothing to allocate
  against; they are drift-only in xdyn by nature, not by omission.
- **Sensor-based Navigation.** Navigation is ground truth. No noise model, no
  sensor fusion.
