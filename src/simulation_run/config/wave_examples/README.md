# Wave examples

The WAM-V of `multi_vehicle_examples/wamv_xdyn.json` in three seas. The
`weather` block and the shared sea are described in `doc/scenario/sea.md`.

| File | Contents |
|---|---|
| `wamv_sea_of_date.json` | The weather of a place and date: Plymouth Sound approaches, 10 August 2024 at 12:00 (waves Hs 0.92 m, Tp 5.2 s, the current profile and a 6.2 m/s wind from Copernicus Marine). One sea, fixed for the run, shared by xdyn and the renderer. Needs the `copernicusmarine` tool and an account on the first run; later runs read the saved copy |
| `wamv_sea_of_date_unshared.json` | The same weather with `"shared": false`: xdyn draws its own random sea of the same spectrum and the renderer its own sea from the wind, so the drawn water and the hull no longer match. For comparison with the shared sea |
| `wamv_beaufort_change.json` | A sea raised from Beaufort 3 to Beaufort 5 at t = 80 s, over 30 s, from the scenario's `schedule`. xdyn and the renderer follow the change together |

```bash
./src/simulation_run/executable/scenario_launch.sh --config wave_examples/wamv_beaufort_change.json
```
