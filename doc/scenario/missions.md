# Missions and behaviour trees

An agent's behaviour is a **behaviour tree** written in its `missions` list.
Leaf nodes run a task (follow waypoints, run a controller, wait); composite
nodes decide in which order their children run. The tasks and their
parameters are in the [Task reference](tasks.md); how the engine works
inside is in [MISSIONS.md](../MISSIONS.md).

## How a mission runs

- Each entry of `missions` is a separate tree. All the trees of an agent run
  side by side, ticked together at the agent's `tick_rate_hz` (default 1 Hz).
- Each tick, a node answers `RUNNING` (not finished), `SUCCESS` or `FAILURE`.
  A tree that has answered `SUCCESS` or `FAILURE` is finished and is not
  ticked again.
- The first tick waits until the agent's vehicle has spawned and its first
  pose has arrived.
- Some tasks never finish: the navigation, guidance, control and allocation
  tasks, and `kinematic_anchor`. They stay `RUNNING` for the whole run, so
  put them as separate trees, or in a `parallel`, never in the middle of a
  `sequence`.
- The launcher logs once when every tree of every agent has finished, the
  never-finishing tasks above excepted.

## Leaf nodes

```json
{ "id": "patrol", "type": "action", "task": "waypoint_follower",
  "params": { "loop": true, "waypoints": [ { "lat": 50.32950, "lon": -4.19400 } ] } }
```

| Field | Meaning |
|---|---|
| `type` | `"action"` or `"condition"`. Both run the same way; by convention a `condition` answers at once and never `RUNNING` |
| `task` | The task's name, see the [Task reference](tasks.md). A task from any installed package can be used |
| `params` | The task's parameters |
| `id` | A free label, for the logs. Optional, need not be unique |

## Composite nodes

| `type` | Fields | Behaviour |
|---|---|---|
| `sequence` | `children` | Runs its children one after another: moves to the next when the current one succeeds, fails as soon as one fails, succeeds when all have succeeded |
| `parallel` | `children`, `success_policy`: `"all"` (default) or `"one"` | Ticks all its unfinished children every tick: fails as soon as one fails, succeeds when all (or one) have succeeded |

Composites nest: a `sequence` can hold a `parallel` and the other way round.

## Spacing out instant actions

A `sequence` moves through all the children that succeed at once within the
same tick. Two `set_wind` back to back therefore apply in the same instant,
and only the second is seen. Put a `wait` between them:

```json
{ "id": "wind_schedule", "type": "sequence", "children": [
    { "id": "calm",      "type": "action", "task": "set_wind", "params": { "x": 0.0, "y": 0.0 } },
    { "id": "hold_calm", "type": "action", "task": "wait",     "params": { "duration_s": 30.0 } },
    { "id": "from_east", "type": "action", "task": "set_wind", "params": { "x": 8.0, "y": 0.0 } }
]}
```

## Example: reach a turbine, then inspect while watching the battery

```json
{
  "id": "patrol_then_inspect",
  "class": "Bluerov2_heavy_inspection",
  "sdf_file": "model-battery.sdf",
  "spawn": { "x": 0.0, "y": 0.0, "z": -10.0, "yaw": 0.0 },
  "tick_rate_hz": 2.0,
  "missions": [
    {
      "id": "patrol_then_inspect_mission",
      "type": "sequence",
      "children": [
        { "id": "reach_turbine", "type": "action", "task": "waypoint_follower",
          "params": {
            "loop": false, "guidance_mode": "bang_bang",
            "waypoints": [ { "lat": 50.32950, "lon": -4.19400 } ]
          } },
        { "id": "inspect_and_monitor", "type": "parallel", "success_policy": "all",
          "children": [
            { "id": "corrosion_crack_inspection", "type": "action",
              "task": "fault_inspection", "params": { "show_window": true } },
            { "id": "led_from_battery", "type": "action",
              "task": "check_battery_state", "params": { "threshold": 80.0 } }
          ] }
      ]
    }
  ]
}
```

Full file: [`src/simulation_run/config/basic_examples/sequence_and_parallel.json`](../../src/simulation_run/config/basic_examples/sequence_and_parallel.json).

---

## A vehicle under the navigation, guidance and control pipeline

The four GNC blocks are four trees that run for the whole scenario, side by
side; Guidance is the only one that changes from one mission to another
(`multi_vehicle_examples/wamv_xdyn.json`):

```json
"missions": [
  { "id": "navigation", "type": "action", "task": "navigation", "params": {} },
  { "id": "path", "type": "action", "task": "guidance_los_polyline",
    "params": { "waypoints": [[60.0, 0.0, 0.0], [60.0, 30.0, 0.0]], "z_start_m": 0.0,
                "u_setpoint_ms": 1.0, "lookahead_m": 10.0 } },
  { "id": "control", "type": "action", "task": "wamv_control", "params": { "n_max_nm": 300.0 } },
  { "id": "allocation", "type": "action", "task": "wamv_allocation", "params": { "t_max_n": 250.0 } }
]
```

See [Navigation, guidance and control tasks](tasks_gnc.md).

## Writing your own task

A task is a Python class registered under the `lotusim.tasks` entry point of
any installed package; scenarios then use it by name, on any agent. See
[Add a behaviour-tree task](../DEVELOPER.md#add-a-behaviour-tree-task).
