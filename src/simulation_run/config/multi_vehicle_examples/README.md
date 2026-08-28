# multi_vehicle_examples

Proof that the generic Navigation/Guidance/Control/Allocation pipeline
(`lotusim_sdk.tasks.{navigation,guidance,control}` +
`lotusim_sdk.tasks.kinematic_allocation`, first built for BlueROV in
`current_examples/`) generalizes to every other vehicle domain -- aerial,
surface and underwater -- not just BlueROV. Each scenario here uses the
same `navigation` / `guidance_hold` / `control`-family task names; only
Allocation (and a thin Control subclass supplying vehicle-specific
damping, where known) is vehicle-specific, by design.

See `../README.md` for the full per-vehicle capability table (Kinematic
vs. xdyn, controllable vs. drift-only, across every vehicle in the fleet)
and an explanation of why scenarios here use `energy.world` while
`current_examples/` uses `bluerov_ekman.world`.

## Scenarios

**Kinematic-controllable** (generic pipeline, no xdyn):
- `x500_kinematic.json` -- aerial (X500). `DomainType::Aerial` routes
  through `KinematicInterface` exactly like Surface/Underwater;
  vertical-motion integration (`vz`) makes altitude hold work the same
  way depth hold does for marine vehicles. A rotor-mixer Allocation
  (Gazebo `MulticopterMotorModel`, alongside PX4 SITL, not replacing it)
  is not yet available in this example set: the controlled trajectory
  is not yet stable.
- `lrauv_kinematic.json` -- underwater (LRAUV), via `lrauv_gnc`'s
  `LrauvControlTask` + the generic `kinematic_allocation`.
- `wamv_kinematic.json` -- surface (WAMV), via `wamv_gnc`'s
  `WamvControlTask` + the generic `kinematic_allocation`.

**xdyn drift-only** (real hydrodynamics + current, no propulsion, no
feedback loop):
- `fremm_drift_only.json`, `wamv_drift_only.json`,
  `commando_drift_only.json`, `pha_drift_only.json`,
  `mine_drift_only.json` -- these vehicles' xdyn YAML declares no
  thruster/rudder actuator at all, so an empty command set is harmless and
  only `navigation` runs, for telemetry. `wamv_drift_only.json` is a
  separate capability from `wamv_kinematic.json`, not a downgrade: WAMV
  genuinely supports both modes, since it has hull dynamics but no
  thruster model to be xdyn-controllable with. `mine_drift_only.json` is
  Mine's only mode, permanently -- it never gets a Kinematic alternative.
- `lrauv_drift_only.json` -- LRAUV's propeller *does* declare an xdyn
  command key, so unlike the vehicles above it needs some command every
  step or xdyn stalls. `lotusim_sdk.tasks.static_command_allocation`
  supplies a fixed near-idle resting command forever, with no feedback
  loop -- same drift-only guarantee, without touching the real Allocation
  gains at all. Not literally 0 rpm: LRAUV's `Kt(J)&Kq(J)` propeller curve
  divides by the commanded rpm, so an exact 0 crashes xdyn outright;
  `propeller(rpm)` is in radians/second despite the key name.

**xdyn controllable**:
- `lrauv_xdyn.json` (`LrauvAllocationTask`) -- surge to a single
  propeller, no yaw/fin actuation. Numerically stable, converging to a
  steady ~0.9 m/s cruise. The vehicle cannot turn or truly station-keep --
  it goes straight once launched and a `guidance_hold` mission can only
  correct depth, not position/heading, since there is no rudder/fin model
  wired into Allocation yet (the vehicle's own YAML has
  `vertical_fins`/`horizontal_fins` hydrodynamic-polar force models fully
  specced out and commented out -- the physics side exists, the
  Allocation/Control wiring to use them does not).

MBARI's LRAUV is a real, publicly documented vehicle with published
hydrodynamic data in the open literature.

`all_vehicles_current_demo.json` spawns one of every vehicle class at once
on `bluerov_ekman.world` -- Commando, Fremm, Pha, WAMV, Mine, LRAUV, and two
`Bluerov2_heavy_pid` -- under a shared ocean current. It checks that every
class still spawns and drifts, rather than demonstrating any one pipeline.

**No dtmb_hull scenarios**: removed (no corresponding Unity asset).
`dtmb_hull_gnc`'s code is untouched in case an asset shows up later, but
no example scenario references it.

## Running

From this repository's root:

```
./src/simulation_run/executable/scenario_launch.sh --config multi_vehicle_examples/x500_kinematic.json
./src/simulation_run/executable/scenario_launch.sh --config multi_vehicle_examples/lrauv_xdyn.json
```

Swap the filename for any scenario listed above. Add `"record_csv": true`
at the scenario's top level to get position telemetry in
`scenario_logs/<timestamp>/csv/<agent>.csv` -- the only way to confirm a
trajectory is correct rather than merely crash-free over the first few
seconds. `record_csv` is already on for `lrauv_xdyn.json`. Every
scenario here runs with `renderer_unity: true`.
