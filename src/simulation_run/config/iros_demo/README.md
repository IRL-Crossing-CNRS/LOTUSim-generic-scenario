# `iros_demo/`

The demo scenario: a full fleet working a 16-turbine farm.

| File | Contents |
|---|---|
| [`demo_iros.json`](demo_iros.json) | 59 agents — 17 WAMV, 15 BlueROV2, 15 X500, 10 LRAUV, plus wind and wake |

```bash
./src/simulation_run/executable/scenario_launch.sh --config iros_demo/demo_iros.json
```

The launcher spreads the 59 agents over several processes on its own (about
ten here). One process cannot hold this fleet: a 20 Hz control loop ran at
0.4 Hz and the vehicles oscillated along their paths. `--shards N` pins the
count — see the repository README.

## What the fleet does

| Behaviour | Agents |
|---|---|
| Orbit a monopile, full GNC loop under xdyn and the Ekman current | `bluerov_orbit_*`, `bluerov_inspect_*` |
| Tour a turbine row, generic GNC loop | `lrauv_tour_*` |
| Orbit a monopile, `waypoint_follower` | `lrauv_orbit_*`, `x500_orbit_*` |
| Run the aisles between two turbine rows or columns | `wamv_aisle_*` |
| Figure of eight around a group of four turbines | `wamv_fig8_*` |
| Drift with the Ekman current, `navigation` only | `wamv_drift_*`, `lrauv_drift_*` |
| Crack detection | `bluerov_crack`, `x500_crack_inspection` |
| YOLO fault inspection | the four `bluerov_inspect_*` |

Six agents carry a detector. The rest fly their trajectory without one.

## Orbit geometry

| | vertices | radius | leg | depth / altitude |
|---|---|---|---|---|
| BlueROV2 orbit | 8 | 50 m | 38 m | 12 m deep |
| BlueROV2 inspection | 8 | 58 m | 44 m | 22 m deep |
| LRAUV orbit | 8 | 66 m | 50 m | 32 m deep |
| X500 orbit | 6 | 12 m | 12 m | 30 m up |

Each leg is long enough for the guidance to settle before the next turn, and
the arrival tolerance is a third of a leg. Shorter legs made the vehicles
overshoot, turn back and overshoot again: they spun in place instead of
advancing.

## Clearance to the foundations

The foundation reaches 38 m from the mast axis. Closest approach along each
group's whole path. The two BlueROV2 rows are measured on a flown run; the
others are the nominal path geometry, which the guidance undercuts slightly
at each corner — expect a few metres less in flight:

| group | closest approach |
|---|---|
| `x500_crack_inspection` | 7.0 m |
| `x500_orbit_*` | 10.4 m |
| `wamv_fig8_*` | 31.8 m |
| `wamv_aisle_*` | 38.0 m |
| `lrauv_tour_*` | 40.0 m |
| `bluerov_orbit_*` | 41.7 m (nominal 46.2) |
| `bluerov_inspect_*` | 49.7 m (nominal 53.6) |
| `lrauv_orbit_*` | 61.0 m |
| drifting agents | 180 m and beyond |

The X500 fly at 30 m altitude, above the water but within the footprint
radius.

## One xdyn process per vessel

Every xdyn agent here carries its own `xdyn_port` (12360-12383), so each gets
its own `xdyn-for-cs`. This is what makes the scenario run in real time: a
shared server integrates its vessels one after another, so the seven WAMV and
three LRAUV that used to share two servers each ran at a fraction of the
speed and held the world back with them. Measured on this scenario, 24 xdyn
agents over 16 servers gave a real-time factor of 0.85; the same 24 over 24
servers gave 1.00.

Add a distinct port to any xdyn agent added later, or it falls back to its
class's shared server.

## Ordering constraint

The launcher starts one xdyn process per agent class, and decides from the
**first** agent of that class in `agents`. A class that mixes xdyn and
Kinematic agents must list an xdyn one first, or its xdyn vessels spawn
without physics. `wamv_drift_ekman` and `lrauv_drift_01` are first for that
reason. This only matters for agents with no `xdyn_port` of their own.

## World

`bluerov_current` is `ekman` and `world_file` is `energy.world`.
`bluerov_ekman.world` carries the 47 deg latitude and 0.05 s step the current
model was validated with, but also a geographic origin 370 km away, and the
patrol files here are lat/lon projected from that origin.

`crack_detector` pins `device=0`, so the host needs a CUDA GPU.
