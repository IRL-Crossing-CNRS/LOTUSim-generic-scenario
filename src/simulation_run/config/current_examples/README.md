# current_examples

Worked examples of the three ocean-current models a BlueROV
scenario can select via the top-level `bluerov_current` key: `ekman`,
`gauss`, `copernicus`, plus the `none` control condition. See
`doc/GNC_MODULAR_ARCHITECTURE.md` for how the pipeline (Navigation ->
Guidance -> Control -> Allocation) and the current injection point fit
together.

See `../README.md` for the full per-vehicle capability table (this
directory only covers BlueROV; other vehicles are in
`../multi_vehicle_examples/`).

For a full evaluation of the current models (Copernicus replays over many
dates, fitted parameters, seeds), see the separate repository
`LOTUSim_current_modelling_evaluation`.

## Scenarios

Two trajectories, each in all four current conditions -- eight scenarios
total. Within a trajectory, the four files are identical except for
`bluerov_current` and the matching `gauss_markov_current` /
`copernicus_current` block, so a diff between them shows exactly what
changes.

**`station_keeping_*.json`** -- one BlueROV holding position at 10 m depth.
**`transect_*.json`** -- one BlueROV running a straight line-of-sight
transect (`bluerov_guidance_los`) that crosses from 3 m to 55 m of
immersion over 200 m of ground track, including the 10 m threshold where
`XdynWebsocket::getNewState()` switches from the Surface to the Underwater
domain.

Current conditions, either trajectory:
- `ekman` -- xdyn's own native Ekman spiral (default if `bluerov_current`
  is omitted).
- `gauss` -- uniform first-order Gauss-Markov process, injected by
  `physics_engine_interface` outside xdyn.
- `copernicus` -- measured depth-profile replay, injected the same way as
  Gauss-Markov. Uses `copernicus_demo_profile.csv` in this directory. The
  `profile` path resolves relative to the config file's own directory, so
  keep the CSV alongside any config that references it. The profile is a
  small synthetic demo, not real Copernicus Marine Service data; see
  `LOTUSim_current_modelling_evaluation` for that.
- `none` -- no-current control condition.

## Running

From this repository's root:

```
./src/simulation_run/executable/scenario_launch.sh --config current_examples/station_keeping_ekman.json
./src/simulation_run/executable/scenario_launch.sh --config current_examples/transect_ekman.json
```

Swap the filename for any of the other seven.

## Also here

`test_current.json` -- one `OceanCurrent` agent on `energy.world`, no
vehicles. Isolates the current field itself, with no GNC pipeline involved.
