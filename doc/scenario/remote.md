# Agents on a remote machine: the scenario file

How a scenario file differs when its agents run on another machine than the simulation.

Remote configs (e.g. `deployment/my_config.json`) have **no**
`world_file`/`renderer_unity` — the remote machine never
launches Gazebo/XDyn/Unity, it only attaches to a world the host already has
running. Everything those headers would have configured instead comes from
`run_agent`'s **CLI flags**:

```bash
python3 -m lotusim_client.run_agent \
    --world energy \
    --config my_config.json \
    --origin 50.32879166666667 -4.195226666666667
```

| Flag | Required | Meaning |
|---|---|---|
| `--world` | ✅ | Must match the `world_name` the host scenario is running (i.e. the `<world name="...">` inside the host's `world_file`, not the file name itself). |
| `--config` / `--json` | one of the two | Path to a JSON file, or an inline JSON string. |
| `--origin LAT LON` | only if any agent uses `lat`/`lon` spawn or `waypoint_follower` | World geographic origin — see [the remote world origin](remote.md#world-origin-must-be-supplied-explicitly). Overrides a config-level `origin`. |

## Config shape — two accepted forms

`run_agent.py` accepts either shape ([the agent keys](scenario_file.md#agent-keys)'s per-agent keys are identical in
both):

**A. The same `"agents": [...]` list form the host uses** — copy a host
scenario's agent entries verbatim (minus `world_file`/
`renderer_unity`, which `run_agent` ignores if present):

```json
{
  "origin": [50.32879166666667, -4.195226666666667],
  "agents": [
    { "id": "patrol", "class": "Bluerov2_heavy", "spawn": {"z": -10.0}, "missions": [ "..." ] }
  ]
}
```

**B. The flat legacy form**, keyed directly by class name at the top level
(what `deployment/my_config.json` uses) — convenient for a single quick
agent, no `"agents"` wrapper or `"id"`/`"class"` needed (the top-level key
*is* the class name, and doubles as the `id`):

```json
{
  "MyBluerov": {
    "origin": [50.32879166666667, -4.195226666666667],
    "nb_agents": 2,
    "tick_rate_hz": 1.0,
    "sdf_file": "model-battery.sdf",
    "poses": [ [0.0, 0.0, -10.0, 0,0,0], [-6.0, 0.0, -10.0, 0,0,0] ],
    "missions": [ "..." ]
  }
}
```

A top-level `origin` key (either shared, as in shape A, or per-agent, as in
shape B) is popped/read before agent processing in either shape; the
`--origin` CLI flag always takes priority over both.

## World origin — must be supplied explicitly

Unlike the host ([the host world origin](scenario_file.md#world-origin)), the remote machine has no world SDF file to read
`<spherical_coordinates>` from, so **you must supply the origin yourself**
whenever an agent needs geo→ENU conversion (`lat`/`lon` spawn, or any
`waypoint_follower` task):

1. Read `latitude_deg`/`longitude_deg` from the world file's
   `<spherical_coordinates>` block (ask whoever runs the host, or check
   `$LOTUSIM_PATH/assets/worlds/<world>.world` if you have access) — for the
   `energy` world it is `50.32879166666667 -4.195226666666667`.
2. Pass it via `--origin LAT LON`, or a config `"origin": [lat, lon]` (top
   level or per-agent) — `--origin` wins if both are given.

**Without an origin, an agent using `lat`/`lon` spawns via a raw `GeoPoint`,
which the host's entity manager places correctly in latitude/longitude but
silently ignores the altitude of** — the agent surfaces instead of spawning
at the intended depth. This is the single most common "my ROV keeps popping
up at the surface" mistake when running remote.

## `reuse_existing` (remote-only)

```json
{ "MyBluerov": { "reuse_existing": true, "spawn": {"z": -10.0} } }
```

If `true` and an entity under this agent's name is **already** present in
Gazebo (detected during the initial discovery window), `run_agent` skips
sending a new `CREATE_CMD` and just re-attaches its mission to the existing
entity — useful when restarting a crashed remote process against a
still-running host, on the **same** machine. Leave it `false` (default) for
the normal case (including any true multi-machine setup): the host is the
single authority on entity names and deconflicts duplicates only through a
fresh `CREATE_CMD`.

## Running it — end to end

```bash
# 1. one-time, per remote machine (see deployment/README.md for the full walkthrough)
source /opt/ros/jazzy/setup.bash
pip install dist/lotusim_sdk-*.whl dist/lotusim_client-*.whl
cd deployment && colcon build && source install/setup.bash
source setup_ros_network.sh   # matches ROS_DOMAIN_ID with the host

# 2. every run
python3 -m lotusim_client.run_agent --world energy --config my_config.json \
    --origin 50.32879166666667 -4.195226666666667
```

Ctrl+C halts every mission's BT roots, sends `DELETE_CMD` for every spawned
agent, and shuts down cleanly (see the remote half of the sequence diagram
in [`ARCHITECTURE.md` §5](../ARCHITECTURE.md#5-host-orchestration-flow-simulation_run)).
Full walkthrough, troubleshooting, and the JSON key reference table:
[`deployment/README.md`](../../deployment/README.md).
