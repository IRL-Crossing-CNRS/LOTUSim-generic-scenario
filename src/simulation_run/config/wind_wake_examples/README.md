# Wind and wake examples

Wind regions, turbine wake, and PX4-controlled aerial drones. All run on
`energy.world`.

| File | Contents |
|---|---|
| `test_wind.json` | One `Wind` agent stepping through six `set_wind` commands separated by `wait` nodes, plus one non-PX4 `X500`. No `Wake` agent. Isolates wind-region behaviour. |
| `x500_px4.json` | One `Wind` agent with a `mirror_global` region, one non-PX4 `X500` running `waypoint_follower` at `z: 50`, and one PX4 `X500` in `manual` control. Smallest scenario exercising PX4 SITL. |
| `px4_manual_wake_flying.json` | One `Wind` agent, one `Wake` agent over 16 turbines at hub altitude 52 m, and two PX4 `X500`s in `manual` control. No mission task; the drones are flown from QGroundControl. |
| `px4_offboard_patrol_test.json` | One `Wind` agent, one `Wake` agent over 7 turbines at hub altitude 85 m, and one PX4 `X500` running `px4_offboard_patrol` across all 7 turbines in sequence. |
| `wake_crossing_demo.json` | One `Wind` agent, one `Wake` agent over 8 turbines at hub altitude 52 m, and two PX4 `X500`s running `px4_offboard_patrol`. `x500_in_wake` flies at `y = -721`, aligned with a turbine's wake; `x500_clear_air` flies the same track at `y = 300`. Records CSV at 10 Hz. |

PX4 agents (`"px4": true`) must spawn at `z: 0`; see the PX4 SITL section of
the repository README for why, and for the `px4_offboard_patrol` parameters.

Turbine hub altitude is per-file: 52 m in `px4_manual_wake_flying.json` and
`wake_crossing_demo.json`, 85 m in `px4_offboard_patrol_test.json`. Within a
file, `wake.turbines[].z` and the flight-path `z` values must match.

The wake model itself is documented in `doc/WAKE_EFFECT.md`; the `wind` and
`wake` JSON blocks are documented in `doc/WRITE_SCENARIO.md`.
