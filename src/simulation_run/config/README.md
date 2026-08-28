# Scenario examples — vehicle capability overview

Two example sets live under this directory, both built on the generic
Navigation/Guidance/Control/Allocation pipeline
(`lotusim_sdk.tasks.{navigation,guidance,control,kinematic_allocation}` --
see `doc/GNC_MODULAR_ARCHITECTURE.md`):

- **`current_examples/`** -- BlueROV only, focused on the four ocean-current
  models (`ekman`/`gauss`/`copernicus`/`none`), station-keeping and
  transect.
- **`multi_vehicle_examples/`** -- one scenario per vehicle domain (aerial,
  surface, underwater) x mode (Kinematic-controllable / xdyn-drift-only),
  proving the same generic pipeline generalizes beyond BlueROV.

## Vehicle capability table

| Vehicle | Domain | Kinematic (no xdyn) | Real xdyn dynamics | Example scenario(s) |
|---|---|---|---|---|
| **BlueROV2 heavy** | Underwater (+ Surface transition) | not used (has real thrusters) | Controllable -- 8-thruster allocator, station-keeping and transects, all 4 current models | `current_examples/*.json` |
| **X500** (aerial drone) | Aerial | Controllable -- generic pipeline, altitude/heading hold via `vz` | not modeled (no xdyn thruster/rotor model; PX4 SITL is the separate non-GNC path, see `x500_px4.json`) | `multi_vehicle_examples/x500_kinematic.json` |
| **WAMV** (surface) | Surface | Controllable -- generic pipeline | Drift-only -- `wamv.yaml` has no thruster model | `multi_vehicle_examples/wamv_kinematic.json`, `wamv_drift_only.json` |
| **LRAUV** (underwater) | Underwater | Controllable -- generic pipeline, depth/heading hold | Controllable, live-verified stable (2026-08-14): single propeller, surge-only, no yaw/fin actuation -- goes straight once launched, `guidance_hold` can only correct depth. Drift-only alternative: propeller held at a near-idle resting command | `multi_vehicle_examples/lrauv_kinematic.json`, `lrauv_xdyn.json`, `lrauv_drift_only.json` |
| **Fremm** (surface) | Surface | not demonstrated (would work like WAMV's Kinematic path) | Drift-only -- hull-only xdyn model, no propulsion | `multi_vehicle_examples/fremm_drift_only.json` |
| **commando** (surface) | Surface | not demonstrated | Drift-only, same pattern as Fremm | `multi_vehicle_examples/commando_drift_only.json` |
| **pha** (surface) | Surface | not demonstrated | Drift-only, same pattern as Fremm | `multi_vehicle_examples/pha_drift_only.json` |
| **Mine** | Underwater | out of scope, permanently | Drift-only only -- by design this vehicle never has propulsion | `multi_vehicle_examples/mine_drift_only.json` |

"Controllable" means a scenario demonstrates the vehicle holding
position/heading or following a track under its own commanded thrust.
"Drift-only" means the vehicle has no propulsion in that mode -- it only
moves under hydrodynamics + current, a correct behavior for hull-only
vessels, not a gap.

**Drift-only and controllable are independent, not tiers of the same
thing.** A vehicle can be xdyn-drift-only permanently, because it has no
thruster model at all (WAMV, commando, pha, Fremm, Mine), or be
xdyn-drift-only alongside an xdyn-controllable Allocation (LRAUV) -- both
cases use the same mechanism, a generic `static_command_allocation` task
publishing a fixed resting command with no feedback loop. Every vehicle
except Mine can switch between Kinematic and xdyn per scenario,
independently of the controllable/drift-only question -- it is the user's
choice, not fixed per vehicle.

**No dtmb_hull scenarios**: dtmb_hull has no corresponding Unity asset, so
its example scenarios were removed rather than kept as
headless/gz-sim-only demos -- every scenario in this directory now runs
with `renderer_unity: true`.

## Why `bluerov_ekman.world` vs. `energy.world`

Every non-BlueROV vehicle's current model (Ekman, in all cases here) is
defined entirely inside that vehicle's own xdyn YAML (`environment
models: - model: ekman current`, with its own `latitude` value), not by
the Gazebo `.world` file. The `.world` file only controls two things:
Gazebo's own physics integration timestep, and a display latitude used
for georeferencing on screen -- neither is what determines which current a
non-BlueROV vehicle actually feels.

`bluerov_ekman.world` is `energy.world` plus exactly two changes, both
made for BlueROV specifically: `latitude_deg=47` (matching
`BlueROV2_current_ekman.yml`'s own latitude, purely so the on-screen
geographic position and the physics agree) and `max_step_size=0.05`
(matching the timestep BlueROV's xdyn model was validated with outside
Gazebo). `current_examples/` uses `bluerov_ekman.world` for that reason.
`multi_vehicle_examples/` uses `energy.world` uniformly -- no vehicle
there depends on BlueROV's specific latitude or timestep.

## Running any scenario

From this repository's root:

```
./src/simulation_run/executable/scenario_launch.sh --config current_examples/station_keeping_ekman.json
./src/simulation_run/executable/scenario_launch.sh --config multi_vehicle_examples/lrauv_kinematic.json
```

Add `"record_csv": true` at the scenario's top level for position telemetry
in `scenario_logs/<timestamp>/csv/<agent>.csv` -- the only way to confirm a
trajectory is sane rather than just "didn't crash in the first 20
seconds". Every file in both directories is a valid `--config` argument.
