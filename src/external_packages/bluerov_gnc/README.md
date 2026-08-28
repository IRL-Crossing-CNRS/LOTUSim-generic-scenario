# bluerov_gnc

BlueROV2 Heavy's GNC (Guidance, Navigation, Control) stack. Registers the
`lotusim.tasks` any BlueROV2 scenario mission list uses -- this package is
not specific to any one experiment; every `bluerov_*` scenario in this
repository (or a downstream one) depends on it.

| task | what it does |
|---|---|
| `bluerov_navigation` | publishes the vehicle's own state for the other tasks to read |
| `bluerov_guidance_hold` | station-keeping: hold a fixed point |
| `bluerov_guidance_los` | line-of-sight guidance to a single waypoint |
| `bluerov_guidance_pure_pursuit` | pure-pursuit guidance to a single waypoint |
| `bluerov_guidance_los_polyline` | line-of-sight over an explicit waypoint list |
| `bluerov_guidance_pure_pursuit_polyline` | pure-pursuit over an explicit waypoint list |
| `bluerov_control` | fixed-gain PID, tracking error -> body-frame wrench |
| `bluerov_allocation` | wrench -> six individual T200 thruster commands |
| `bluerov_metrics_recorder` | subscribes only; writes the CSV/summary below |

The guidance tasks (`guidance_tasks.py`) are thin BlueROV-named aliases over
the vehicle-agnostic Guidance task in `lotusim_sdk/tasks/guidance.py`, which
is where cross-track/along-track error is actually computed -- reusable
across vehicles, not duplicated here.

## Metrics recorder (`metrics_recorder_task.py`)

Writes one CSV row per navigation update and a JSON summary at the end, to
`results/<output_dir>/<run_name>/<agent_id>.csv` /`_summary.json`. Columns:
pose (NED), body-frame velocity, guidance targets, tracking errors
(cross-track, depth, heading, position), commanded wrench, the six thruster
commands, and thruster power/energy (below). It only subscribes, so it
cannot alter a run's dynamics, and it never stops itself -- there is no
scripted end condition, so a mission runs until the launcher is stopped.

## Thruster power model (`power.py`)

**Energy is a model of the six T200 thrusters' electrical draw from their
commanded thrust (propeller momentum theory, `P = K|T|^1.5 + P_idle`), not a
battery measurement** -- the simulator has no battery model, nothing here
touches the vehicle's dynamics, and absolute values carry a calibration
error of up to ~17% against the public T200 curve (see the module
docstring). Ratios between two conditions run on the same model are
unaffected by that calibration error; an absolute number is not comparable
across a different `K`/`P_idle` choice without refitting first.

Useful standalone, independent of any particular experiment, for anything
that wants a cheap proxy for a BlueROV2 mission's energy cost from its
thruster commands alone -- `thruster_power(thrust_N)` /
`total_power(thrusts)` take plain newtons in, watts out, no simulator state
needed.
