# The scenario file

A generic scenario is one JSON file: the world to start, the agents to spawn
and what each of them does. This page lists every key of the file and of an
agent; what agents do is described in [Missions](missions.md) and the
[Task reference](tasks.md).

## A minimal scenario

```json
{
  "world_file": "energy.world",
  "renderer_unity": true,
  "agents": [
    { "id": "rov", "class": "Bluerov2_heavy",
      "spawn": { "x": 0.0, "y": 0.0, "z": -10.0, "yaw": 0.0 },
      "missions": [
        { "id": "patrol", "type": "action", "task": "waypoint_follower",
          "params": { "loop": true, "waypoints": [ { "lat": 50.32950, "lon": -4.19400 } ] } }
      ] }
  ]
}
```

Scenario files live in `src/simulation_run/config/`; pass one to the launcher
with `--config` ([launch options](../LAUNCH_OPTIONS.md)).

## Top-level keys

| Key | Type | Default | Meaning |
|---|---|---|---|
| `world_file` | string | `""` | Gazebo world file, from the core's `assets/worlds/`, e.g. `"energy.world"`. Its `<spherical_coordinates>` give the geographic origin used to place lat/lon waypoints ([world origin](#world-origin)) |
| `agents` | list | `[]` | The agents, see [agent keys](#agent-keys). A dict keyed by class name is also accepted ([legacy form](#legacy-dict-form-of-agents)) |
| `renderer_unity` | bool | `false` | Start the ROS–Unity bridge and the Unity player |
| `record_csv` | bool or object | `false` | Record every agent to CSV, see [Recording](recording.md) |
| `record_bag` | bool or list of topics | `false` | Record a ROS 2 bag into `$LOG_DIR/bag`: every topic, or the listed ones, see [Recording](recording.md) |
| `waves` | object | none | One sea state for every xdyn vessel and the renderer, see [Waves and weather](sea.md) |
| `weather` | object | none | The sea, current and wind from a Beaufort force or from a place and date, see [Waves and weather](sea.md) |
| `bluerov_current` | string | `"ekman"` | Ocean current felt by `Bluerov2_heavy_current` agents: `"ekman"`, `"gauss"`, `"copernicus"` or `"none"`, see [Ocean current](current.md) |

The environment itself (wind, turbine wakes, a current for Kinematic
vehicles) is not a top-level key: it is set by environment agents (`Wind`,
`Wake`, `OceanCurrent`) in the `agents` list, see [Wind and wake](wind.md)
and [Ocean current](current.md).

## Agent keys

Each entry of the `agents` list (host) or config object (remote, [Remote agents](remote.md)) accepts:

| Key | Type | Default | Meaning |
|---|---|---|---|
| `id` | string | class name, lowercased | Base name for instances: `nb_agents=3` with `"id": "patrol"` spawns `patrol0`, `patrol1`, `patrol2`. |
| `class` (or `type`) | string | — (required) | Resolved to a Python class via the `lotusim.agents` entry point (e.g. `"Bluerov2_heavy"`, `"Bluerov2_heavy_inspection"`, `"Wamv"`, `"X500"`, any custom agent). Matched once both names are lowercased and CamelCase is split with underscores: `Bluerov2Heavy` and `Bluerov2_heavy` both find `bluerov2_heavy`. |
| `nb_agents` | int | `1` | Number of instances to spawn of this entry. |
| `sdf_file` | string | `""` (model's default `model.sdf`) | Picks a specific SDF variant inside the model's asset folder, e.g. `"model-battery.sdf"` (needed for `check_battery_state`/light-actuator demos). |
| `xdyn` | bool | `false` | Enables the XDyn physics connection for this agent (needs a non-`Aerial` domain and the class's `XDYN_PORT`). A `waypoint_follower` task overrides this to a native `Kinematic` connection regardless (see [`MISSIONS.md` §4.3](../MISSIONS.md#43-worked-example--waypointfollowertask)). |
| `xdyn_port` | int | — (the class's shared port) | Gives this agent its own `xdyn-for-cs` process on that port instead of the one shared by its class. One server integrates its vessels one after another, so several xdyn agents on one server slow the whole world down. Give every xdyn agent of a class a distinct port, or none of them. |
| `spawn` | object `{x,y,z,roll,pitch,yaw}` | all fields `0.0` | Explicit ENU spawn pose. Highest-priority explicit form. |
| `pose` | `[x,y,z,roll,pitch,yaw]` | — | Same as `spawn`, list form (remote only, see [Remote agents](remote.md)). |
| `poses` | list of the above | — | One entry per instance index (`poses[i]` for the i-th of `nb_agents`); falls back if the index is out of range (host: random pose; remote: `poses[0]`). |
| `tick_rate_hz` | float | `1.0` | BT mission tick frequency for this agent — independent per agent. |
| `missions` | list of BT nodes | `[]` | The behaviour tree(s) — see [Missions](missions.md). Omit entirely for a bare spawn with no behaviour. |

Any other key is passed through to the agent class's constructor as
`**kwargs` and/or is specific to the launcher (host vs. remote, [host](scenario_file.md) vs. [remote](remote.md)).

---

### Keys of some agent classes

| Key | Classes | Type | Meaning |
|---|---|---|---|
| `px4` | `X500` | bool, default `false` | Fly the drone with PX4 SITL instead of LOTUSim's guidance, see [PX4 drones](../PX4_DRONES.md) |
| `px4_control` | `X500` with `px4` | `"manual"` or `"offboard"`, default `"manual"` | Flown from QGroundControl, or by the `px4_offboard_patrol` task |
| `gauss_markov_current` | xdyn vessels | object | A random current injected on this vessel: `mean_x`, `mean_y` (m/s, default 0), `tau` (correlation time, s, default 60), `std_dev` (m/s, default 0), `seed` (default 0). See [Ocean current](current.md) |
| `copernicus_current` | xdyn vessels | object | Replay of a current depth profile: `profile`, a CSV with columns `depth_m, vx_north_ms, vy_east_ms`, relative to the scenario file. Exclusive with `gauss_markov_current` |
| `ais` | legacy dict form | object | `patrol_file`: one patrol file, or a list of them (one per agent, each `{"file", "loop"}` or a path), that the agents follow |

## Spawn pose

Where an agent appears. Put **one** of these keys per agent:

| Key | Form | Where |
|---|---|---|
| `spawn` | `{"x", "y", "z", "roll", "pitch", "yaw"}`, ENU, missing fields 0 | host and remote |
| `poses` | `[[x, y, z, roll, pitch, yaw], ...]`, one per instance of `nb_agents` | host and remote |
| `pose` | `[x, y, z, roll, pitch, yaw]` | remote |
| `lat`, `lon` | geographic position, projected with the [world origin](#world-origin) | remote |

With none of them, the host draws a random pose in the vehicle's domain, the
remote uses `[0, 0, 0, 0, 0, 0]`. Depth and altitude are the `z` of the pose:
negative under water, positive in the air. On the remote, `lat`/`lon` without
a world origin spawns at the right position but at the surface, whatever the
depth (see [the remote world origin](remote.md#world-origin-must-be-supplied-explicitly)).

## World origin

The host reads the geographic origin of the world from the `<spherical_coordinates>`
of `world_file` and gives it to every agent before its missions start, so
lat/lon waypoints need no configuration. A remote agent has no world file and
needs the origin given explicitly ([the remote world origin](remote.md#world-origin-must-be-supplied-explicitly)).

## Legacy dict form of `agents`

`agents` can also be a **dict** keyed by class name:

```jsonc
"agents": {
  "Bluerov2_heavy": { "nb_agents": 2, "poses": [[...], [...]], "xdyn": true }
}
```

`basic_examples/empty.json` uses it. Write new scenarios with the list form:
waypoint files and spawn poses given inside missions are only read from it.

## Host and remote scenarios

The agent keys and the missions are the same whether the agents run on the
simulation machine (the host, started by `scenario_launch.sh`) or on another
machine (`lotusim_client.run_agent`): a mission written for one works in the
other. Only the top-level keys differ: `world_file`, `renderer_unity` and the
others tell the host what to start, and a remote machine starts nothing. It
takes the world to join and its origin from command-line flags instead, see
[Remote agents](remote.md).
