# Facet inspection demos

Full-fleet turbine-inspection scenarios on `energy.world`, with a 16-turbine
farm at hub altitude 52 m. Both record CSV at 10 Hz.

| File | Contents |
|---|---|
| `demo_facet.json` | `Wake` agent plus 5 `Bluerov2_heavy_inspection`, 5 `Wamv_inspection`, and 5 `X500_inspection` agents, each running `waypoint_follower` and `fault_inspection` at turbines 1, 5, 9, 10 and 13. No `Wind` agent and no xdyn. |
| `demo_facet_physics.json` | `Wind` and `Wake` agents, 5 `Bluerov2_heavy_pid` under the GNC pipeline, 5 `Wamv`, 5 `X500_inspection`, and the two PX4 `X500`s from `wind_wake_examples/wake_crossing_demo.json` running `px4_offboard_patrol`. |

`demo_facet.json` is the kinematic predecessor of `demo_facet_physics.json`:
same fleet layout and same farm, with `xdyn: false` throughout and no wind.

Both read patrol files from `../waypoints/`. Running
`demo_facet_physics.json` requires a built PX4 checkout; see the PX4 SITL
section of the repository README.
