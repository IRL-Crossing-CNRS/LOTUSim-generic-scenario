# Scenario examples

Every scenario JSON in this tree is a valid `--config` argument. Paths are
resolved relative to this directory:

```bash
./src/simulation_run/executable/scenario_launch.sh --config basic_examples/waypoint_solo.json
./src/simulation_run/executable/scenario_launch.sh --config current_examples/station_keeping_ekman.json
./src/simulation_run/executable/scenario_launch.sh --config wind_wake_examples/wake_crossing_demo.json
```

## Directories

| Directory | Contents |
|---|---|
| [`basic_examples/`](basic_examples/) | Behaviour-tree fundamentals on a single BlueROV, plus an empty world |
| [`current_examples/`](current_examples/) | Ocean-current models with a BlueROV2 under PID |
| [`wind_wake_examples/`](wind_wake_examples/) | Wind regions, turbine wake, and PX4 aerial drones |
| [`multi_vehicle_examples/`](multi_vehicle_examples/) | One scenario per vehicle class and control mode |
| [`facet_demo/`](facet_demo/) | Full-fleet turbine-inspection demos |
| [`waypoints/`](waypoints/) | Patrol waypoint data files referenced by `waypoints_file`; not scenarios |

Add `"record_csv": true` at a scenario's top level to write position
telemetry to `scenario_logs/<timestamp>/csv/<agent>.csv`. That telemetry is
the only way to confirm a trajectory is correct rather than merely
crash-free over the first few seconds.

`current_examples/` and `multi_vehicle_examples/` are built on the generic
Navigation/Guidance/Control/Allocation pipeline
(`lotusim_sdk.tasks.{navigation,guidance,control,kinematic_allocation}`, see
`doc/GNC_MODULAR_ARCHITECTURE.md`).

## Vehicle capability table

| Vehicle | Domain | Kinematic (no xdyn) | Real xdyn dynamics | Example scenario(s) |
|---|---|---|---|---|
| **BlueROV2 heavy** | Underwater (+ Surface transition) | not used (has real thrusters) | Controllable -- fixed allocator over the 6 modelled thrusters of the 8 (4 vectored + 2 vertical; roll and pitch are not commandable), station-keeping and transects, all 4 current models | `current_examples/*.json` |
| **X500** (aerial drone) | Aerial | Controllable -- generic pipeline, altitude/heading hold via `vz` | not modeled (no xdyn thruster/rotor model; PX4 SITL is the separate non-GNC path, see `wind_wake_examples/x500_px4.json`) | `multi_vehicle_examples/x500_kinematic.json` |
| **WAMV** (surface) | Surface | Controllable -- generic pipeline | Drift-only -- `wamv.yaml` has no thruster model | `multi_vehicle_examples/wamv_kinematic.json`, `wamv_drift_only.json` |
| **LRAUV** (underwater) | Underwater | Controllable -- generic pipeline, depth/heading hold | Controllable, stable: single propeller, surge-only, no yaw/fin actuation -- goes straight once launched, `guidance_hold` can only correct depth. Drift-only alternative: propeller held at a near-idle resting command | `multi_vehicle_examples/lrauv_kinematic.json`, `lrauv_xdyn.json`, `lrauv_drift_only.json` |
| **Fremm** (surface) | Surface | not demonstrated (would work like WAMV's Kinematic path) | Drift-only -- hull-only xdyn model, no propulsion | `multi_vehicle_examples/fremm_drift_only.json` |
| **commando** (surface) | Surface | not demonstrated | Drift-only, same pattern as Fremm | `multi_vehicle_examples/commando_drift_only.json` |
| **pha** (surface) | Surface | not demonstrated | Drift-only, same pattern as Fremm | `multi_vehicle_examples/pha_drift_only.json` |
| **Mine** | Underwater | out of scope, permanently | Drift-only only -- by design this vehicle never has propulsion | `multi_vehicle_examples/mine_drift_only.json` |

"Controllable" means a scenario demonstrates the vehicle holding
position/heading or following a track under its own commanded thrust.
"Drift-only" means the vehicle has no propulsion in that mode -- it only
moves under hydrodynamics + current, a correct behavior for hull-only
vessels, not a gap.

Drift-only and controllable are independent, not tiers of the same thing. A
vehicle can be xdyn-drift-only permanently, because it has no thruster model
at all (WAMV, commando, pha, Fremm, Mine), or be xdyn-drift-only alongside an
xdyn-controllable Allocation (LRAUV). Both cases use the same mechanism: a
generic `static_command_allocation` task publishing a fixed resting command
with no feedback loop. Every vehicle except Mine can switch between Kinematic
and xdyn per scenario, independently of the controllable/drift-only question.

There are no dtmb_hull scenarios. dtmb_hull has no corresponding Unity asset,
so its example scenarios were removed rather than kept as headless
gz-sim-only demos; every scenario in this tree runs with
`renderer_unity: true`.

## Why `bluerov_ekman.world` vs. `energy.world`

Every non-BlueROV vehicle's current model (Ekman, in all cases here) is
defined inside that vehicle's own xdyn YAML (`environment models: - model:
ekman current`, with its own `latitude` value), not by the Gazebo `.world`
file. The `.world` file controls two things: Gazebo's own physics integration
timestep, and a display latitude used for georeferencing on screen. Neither
determines which current a non-BlueROV vehicle feels.

`bluerov_ekman.world` is `energy.world` plus two changes, both made for
BlueROV specifically: `latitude_deg=47` (matching
`BlueROV2_current_ekman.yml`'s own latitude, so the on-screen geographic
position and the physics agree) and `max_step_size=0.05` (matching the
timestep BlueROV's xdyn model was validated with outside Gazebo).
`current_examples/` uses `bluerov_ekman.world` for that reason.
`multi_vehicle_examples/` uses `energy.world` uniformly, as no vehicle there
depends on BlueROV's latitude or timestep.
