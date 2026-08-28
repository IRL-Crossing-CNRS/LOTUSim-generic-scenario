# Basic examples

Behaviour-tree fundamentals. Each scenario runs a single vehicle on
`energy.world` (or an accelerated variant) with `xdyn: false`, so the focus
is the BT structure rather than the physics.

| File | Contents |
|---|---|
| `empty.json` | No agents. Starts the world and the renderer only. Uses the legacy dict form of `agents`. |
| `waypoint_solo.json` | One `Bluerov2_heavy` running `waypoint_follower` over three inline waypoints, looping. |
| `waypoint_solo_file.json` | The same patrol, with waypoints read from `waypoints_file` instead of an inline list. |
| `waypoint_solo_accelerated5x.json` | `waypoint_solo.json` on `energy_accelerated5x.world` (5x real time, wall clock). |
| `waypoint_solo_accelerated50x.json` | `waypoint_solo.json` on `energy_accelerated50x.world` (50x real time, pose clock). |
| `inspection_and_battery.json` | One `Bluerov2_heavy_inspection` running `fault_inspection` and `check_battery_state` under a `parallel` node with a `success_policy`. |
| `sequence_and_parallel.json` | One `Bluerov2_heavy_inspection`: `waypoint_follower`, then `fault_inspection` and `check_battery_state` in parallel. Demonstrates `sequence` composed with `parallel`. |
| `custom_task_demo.json` | One `CustomTaskDemoAgent`. Scaffold for a task that is not built in. |

`waypoint_solo_file.json` reads `../waypoints/waypoint_windturbine1.json`.
The path is resolved relative to the scenario JSON's own directory, so a
patrol file outside this folder must be referenced with `../`.

See `doc/ACCELERATED_SIMULATION.md` for the two accelerated scenarios and
`doc/MISSIONS.md` for the BT node types.
