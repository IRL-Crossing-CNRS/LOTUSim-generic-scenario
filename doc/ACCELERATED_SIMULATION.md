# Running a scenario faster than real time

How to run a LOTUSim world at a real-time factor (RTF) greater than 1, and how
the waypoint guidance stays correct when you do.

> Scope: the RTF mechanism and the `guidance_clock` mission parameter. For the
> full scenario JSON reference see [the generic scenario reference](scenario/README.md); for
> the BT task lifecycle see [`MISSIONS.md`](MISSIONS.md).

---

## 1. Setting the real-time factor

Gazebo advances simulated time in fixed steps of `max_step_size` seconds and, by
default, throttles so that one simulated second takes one wall-clock second
(RTF 1). The target RTF is set in the world's `<physics>` block:

```xml
<physics type="ode">
    <max_step_size>0.1</max_step_size>   <!-- sim seconds per physics step -->
    <real_time_factor>5</real_time_factor> <!-- target: 5 sim-s per wall-s -->
</physics>
```

`real_time_factor` is a **target**, not a guarantee. Gazebo runs steps as fast
as it can up to that cap; the achieved RTF equals the target only if the machine
can compute the steps that fast. `max_step_size` is unchanged by acceleration —
only the wall-clock rate of stepping changes.

The provided example worlds (`assets/worlds/` in the host repo) keep the world
**name** `energy`, so all `/energy/...` topics and the `lotusimenergy` Unity
build work unchanged; only the physics rate differs:

| World file | `real_time_factor` |
|---|---|
| `energy.world` | 1 |
| `energy_accelerated5x.world` | 5 |
| `energy_accelerated50x.world` | 50 |

A world file referenced by a scenario with `renderer_unity: true` needs an entry
in `WORLD_UNITY_BASENAMES` (keyed by file name) in
`simulation_run/executable/scenario_launch.sh`.

## 2. Why the guidance clock matters at high RTF

Pose integration runs host-side in `KinematicInterface` using Gazebo's sim-time
step, so vessel motion is correct at any RTF on its own. The agent-side
kinematic waypoint guidance (`WaypointFollowerTask`) produces the velocity and
heading set-points; it reads the pose stamp on `/<world>/poses` (Gazebo sim
time) for its control `dt`, so its velocity/heading ramps are correct at any
RTF.

What still depends on RTF is **how often** the control loop runs, set by
`guidance_clock`:

- **`"wall"` (default)** — a wall-clock ROS timer at `control_rate_hz`. Control
  updates per simulated second = `control_rate_hz / RTF`. At the default 20 Hz
  this is 20 updates/sim-s at RTF 1 but ~1 update/sim-s at RTF 20; below that,
  waypoint arrival is sampled too rarely and turns overshoot.

- **`"pose"`** — one control step per `/<world>/poses` message, i.e. once per
  physics step (`1 / max_step_size` = 10 Hz of sim time for the energy world).
  This rate is independent of RTF, so tracking is unchanged at any RTF.
  `control_rate_hz` is ignored in this mode.

Set it in the task `params`:

```json
"params": {
  "guidance_clock": "pose",
  "loop": true,
  "waypoints": [ ... ]
}
```

Rule of thumb: `"wall"` is fine up to ~20× (or raise `control_rate_hz`
proportionally); use `"pose"` for higher RTF. In `"pose"` mode the remaining
limit is Gazebo's physics throughput — the machine must sustain `RTF / max_step_size`
control steps per wall-second per agent.

## 3. Example scenarios

| Scenario | World | `guidance_clock` |
|---|---|---|
| `basic_examples/waypoint_solo.json` | `energy.world` (1×) | wall |
| `basic_examples/waypoint_solo_accelerated5x.json` | `energy_accelerated5x.world` (5×) | wall |
| `basic_examples/waypoint_solo_accelerated50x.json` | `energy_accelerated50x.world` (50×) | pose |

Launch:

```bash
./src/simulation_run/executable/scenario_launch.sh --config basic_examples/waypoint_solo_accelerated50x.json
```

## 4. Checking the achieved RTF

Read it from the world's own statistics topic while the scenario runs:

```bash
gz topic -e -t /world/energy/stats | grep real_time_factor
```

Use the topic of your world, `/world/<world>/stats`. The bare `/stats` topic
is shared by every world of the run, the aerial world included, and shows
whichever one answers.

A world below its target is limited by the machine: many xdyn hulls, sensors
or agents slow each step down. Lower the target, or the load, until the
achieved factor holds.
