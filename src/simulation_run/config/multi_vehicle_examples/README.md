# Multi-vehicle examples

One scenario per vehicle class and mode. They all use the same generic
navigation, guidance, control and allocation tasks as the BlueROV
(`doc/GNC_MODULAR_ARCHITECTURE.md`); only the allocation, and for some
vehicles a thin control subclass, is specific to a vehicle.

The table of what each vehicle can do, in each mode, is in
[`../README.md`](../README.md).

```bash
./src/simulation_run/executable/scenario_launch.sh --config multi_vehicle_examples/wamv_xdyn.json
```

## Kinematic: following a path without hydrodynamics

The vehicle follows its commanded velocity exactly.

| File | Vehicle | Tasks |
|---|---|---|
| `x500_kinematic.json` | X500 drone | Generic pipeline; altitude held through the vertical velocity |
| `lrauv_kinematic.json` | LRAUV | `lrauv_gnc`'s `LrauvControlTask` and the generic `kinematic_allocation` |
| `wamv_kinematic.json` | WAM-V | `wamv_gnc`'s `WamvControlTask` and the generic `kinematic_allocation` |

The X500 under PX4 is a separate path: `wind_wake_examples/x500_px4.json`.

## xdyn, controlled

| File | Vehicle | What it does |
|---|---|---|
| `wamv_xdyn.json` | WAM-V 16 at full scale | Follows a three-leg, 150 m path at 1 m/s in a JONSWAP sea (Hs 0.5 m, Tp 3 s) and the Ekman current, then holds the last point. Two stern engines: their sum gives surge, their difference yaw. See `doc/WAMV_MODEL.md` |
| `lrauv_xdyn.json` | LRAUV | Cruises at about 0.9 m/s on its single propeller. It has no fin control: it holds its depth but cannot turn |

## xdyn, drifting

No propulsion: the vehicle moves only with the waves and the current, and
only `navigation` runs, for the telemetry.

| File | Vehicle | Notes |
|---|---|---|
| `fremm_drift_only.json`, `commando_drift_only.json`, `pha_drift_only.json` | Surface hulls | Their xdyn models have no actuator |
| `mine_drift_only.json` | Mine | Has no propulsion in any mode |
| `wamv_drift_only.json` | WAM-V | Both engines held at zero thrust |
| `lrauv_drift_only.json` | LRAUV | The propeller held at 15 rpm by `static_command_allocation`: its propeller model needs a command every step and cannot take exactly 0 rpm |

`all_vehicles_current_demo.json` spawns one vehicle of every class
(Commando, Fremm, Pha, WAM-V, Mine, LRAUV and two `Bluerov2_heavy_current`)
on `bluerov_ekman.world` under one current: a quick check that every class
spawns and drifts.

Every scenario here records CSV telemetry and runs with Unity.
