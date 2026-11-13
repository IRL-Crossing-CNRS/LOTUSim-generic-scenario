# Facet inspection demos

Full-fleet turbine-inspection scenarios on `energy.world`, with a 16-turbine
farm at hub altitude 52 m. All record CSV at 10 Hz.

| File | Contents |
|---|---|
| `demo_facet.json` | `Wake` agent plus 5 `Bluerov2_heavy_inspection`, 5 `Wamv_inspection`, and 5 `X500_inspection` agents, each running `waypoint_follower` and `fault_inspection` at turbines 1, 5, 9, 10 and 13. No `Wind` agent and no xdyn. |
| `demo_facet_physics.json` | `Wind` and `Wake` agents, 5 `Bluerov2_heavy_current_inspection` under the GNC pipeline, 5 `Wamv`, 5 `X500_inspection`, and the two PX4 `X500`s from `wind_wake_examples/wake_crossing_demo.json` running `px4_offboard_patrol`. |
| `wamv_waves.json` | `demo_facet.json` with the 5 WAM-Vs on xdyn (one xdyn process each), following the same patrols with the WAM-V GNC pipeline, `fault_inspection` kept. One sea for every hull and the Unity 6 renderer: `"weather": {"beaufort": 3, "follow_wind": true}`, so the Unity wind sliders raise or calm the sea during the run. |
| `demo_facet_october2026.json` | The fleet of `iros_demo/demo_iros.json` (59 agents: 17 WAM-V, 15 BlueROV2, 15 X500, 10 LRAUV, wind and wake) in one fixed sea: `"weather": {"beaufort": 3}`, evaluated by each xdyn itself, wind 5 m/s, and the ten patrolling WAM-Vs (`wamv_aisle_*`, `wamv_fig8_*`) on xdyn with the WAM-V GNC pipeline, three laps of their IROS paths. 34 xdyn agents, one xdyn process each (ports 12360-12393). The Unity 6 player draws the waves, the farm's submarine cables and the two cracks the crack detectors look at. The 15 X500 are PX4 drones (`"px4": true`, `px4_offboard_patrol`): PX4 SITL flies each one, so the wind and the turbine wakes push them, and they take off from the sea surface. Every vehicle spawns with its `model-battery.sdf`, so each one publishes `/<world>/<agent>/battery/state` (the Unity battery panel, the wake agent's LCOE). |
| `demo_facet_light_october2026.json` | `demo_facet_october2026.json` for a 16 GB machine: same farm, sea and two cracks, 19 agents instead of 59. It keeps wind, wake, the two crack detectors, `bluerov_gnc_ekman`, 3 orbiting BlueROV2, 4 patrolling WAM-Vs, 4 LRAUV and 3 orbiting X500, and drops the four `bluerov_inspect_*` (each loads its own YOLO model) and the drifting vehicles. 8 xdyn processes instead of 34, 4 PX4 drones instead of 15, 2 YOLO models instead of 6. |

`demo_facet.json` is the kinematic version of `demo_facet_physics.json`:
same fleet layout and same farm, with `xdyn: false` throughout and no wind.

`demo_facet_physics.json` sets `bluerov_current: ekman` explicitly rather than
relying on the launcher's default. It stays on `energy.world` even so:
`bluerov_ekman.world` carries the 47 deg latitude and 0.05 s step the Ekman
model was validated with, but also a geographic origin 370 km away, and the
patrol files here are lat/lon projected from that origin. The current
therefore runs with the 0.1 s step of `energy.world`.

Both read patrol files from `../waypoints/`. Running
`demo_facet_physics.json`, `demo_facet_october2026.json` or `demo_facet_light_october2026.json` requires a built
PX4 checkout; see `doc/PX4_DRONES.md`.
