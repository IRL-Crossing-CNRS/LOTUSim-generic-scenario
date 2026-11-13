# Recording a run

CSV telemetry of every agent, written while the scenario runs, and on request a
[ROS 2 bag](#sensor-streams-a-ros-2-bag) of the sensor streams.

A pure **observer** node (`simulation_run.csv_recorder.CsvRecorder`) — it
spawns nothing and is not attached to any agent, so it fits the distributed
multi-agent model: recording is a property of whoever wants the data, not of
the vehicles being recorded. Add it to the host executor with a single
top-level key:

```jsonc
"record_csv": true
// or, with options:
"record_csv": {
  "rate": 5.0,
  "outdir": "my_csv_dir",
  "prefix": "run1_",
  "ref_lat": 50.32879166666667,
  "ref_lon": -4.195226666666667,
  "ref_alt": 0.0
}
```

| Field | Type | Default | Meaning |
|---|---|---|---|
| `rate` | float (Hz) | `2.0` | Sampling rate. |
| `outdir` | string | `$LOG_DIR/csv` if `scenario_launch.sh` set `LOG_DIR`, else `./csv_logs_<world>_<timestamp>/` | Output directory. |
| `prefix` | string | `""` | Prepended to every per-agent filename. |
| `ref_lat`, `ref_lon`, `ref_alt` | float | `energy.world`'s origin | Reference point for the recorded `lat`/`lon` columns (WGS84 local tangent-plane about this point) — should match the running world's `<spherical_coordinates>`. |

One CSV file **per agent** — `<outdir>/<prefix><agent_name>.csv` — populated
by subscribing to `/<world>/poses` (all agents, host-authoritative) and
auto-discovering every `/<world>/<agent>/battery/state` topic (only present on
agents spawned with a battery sensor, e.g. `sdf_file: "model-battery.sdf"`).

**The column set is per-agent, not fixed** — each group below is only
included for an agent that actually has the thing it describes, decided from
the scenario config (not from whatever topic happens to arrive first at
runtime).

An agent may declare an explicit equipment manifest to make this decision
(and `summary.csv`'s `sensors`/`actuators` columns) unambiguous, rather than
relying on the `sdf_file`/mission-params heuristics below:

```jsonc
{
  "id": "bluerov1",
  "sensors": ["battery", "sonar"],
  "actuators": ["thrusters"],
  ...
}
```

This is a LOGGING-side manifest, read by
`csv_recorder._agent_capabilities_from_scenario` — it does not itself equip
the agent (that's still `sdf_file` for battery and the mission
`task`/`params` for sonar, unchanged); declaring `"sonar"` here without also
giving the agent a mission whose `params` sets `sonar_range_m` just makes the
CSV lie about what it has. When `"sensors"`/`"actuators"` are absent
(scenarios written before this field existed), battery/sonar columns fall
back to the pre-existing heuristics (`sdf_file` naming,
`params.sonar_range_m`) exactly as before.

```txt
# base — every agent
agent_name, sim_time_s,
pos_x, pos_y, pos_z, lat, lon,
orient_x, orient_y, orient_z, orient_w

# + current — only scenarios declaring an OceanCurrent agent
current_vx_mps, current_vy_mps, current_vz_mps

# + battery — only agents spawned with a battery sensor
battery_voltage, battery_charge_ah, battery_capacity_ah,
battery_percentage, battery_status

# + mission — only agents with a real navigation mission
mission_id, task_type,
target_waypoint_idx, target_x, target_y, target_lat, target_lon,
distance_to_target_m, cross_track_error_m, along_track_progress_pct,
waypoint_arrived, arrival_error_m, arrival_error_pct, mission_complete

# + sonar — only agents whose mission sets params.sonar_range_m
sonar_range_m, sonar_contact, sonar_distance_m
```

`current_vx_mps`/`current_vy_mps`/`current_vz_mps` are the configured
`OceanCurrent` agent's vector ([Ocean current](current.md)), repeated on every row so one agent's
file is self-contained for "what current was it under" — not something
computed per-agent, the same global value on every row of every agent's
file. `current_vz_mps` reports the agent's configured `z`, but has no
physical effect yet — the current model is horizontal-only ([Ocean current](current.md)).

These three columns exist **only when the scenario declares an
`OceanCurrent` agent**. xdyn's own current models — `bluerov_current:
ekman` in the vehicle YAML, and the gauss/copernicus models inside the
physics plugin — never pass through that agent and are not visible to the
recorder, so a run under one of them gets no current columns at all. Read
the current's effect there from the trajectory itself (the crab angle and
`cross_track_error_m`) or from a per-vehicle metrics recorder. `summary.csv`
carries its `ocean_current_*` columns under the same condition.

`sonar_range_m`/`sonar_contact`/`sonar_distance_m` are the fake sonar ([Task reference](tasks.md),
`waypoint_follower_avoidance`): a ground-truth distance to the nearest agent
whose scenario `"class"` is `"mine"` (case-insensitive), recomputed
independently by the recorder every sample — not read from the task's own
internal state. `sonar_distance_m` is only populated when `sonar_contact`
is `1` (blank otherwise) — like a real sonar, there's no reading at all for
something outside detection range. Only present at all for agents whose
active mission sets `params.sonar_range_m` — e.g. a mine's CSV has no sonar
columns (it has no sonar), and neither does a plain `waypoint_follower`
vehicle with no `sonar_range_m` set. Deliberately **no identity column** —
range only, like a real sonar; the recorder uses the nearest object's true
name internally (to pick the closest one and to count distinct contacts for
`summary.csv`) but never writes it to a column here. The vehicle's own task
follows the same rule: `WaypointFollowerAvoidanceTask`'s terminal log line
(`<vehicle>: sonar contact at <dist> m ... - engaging avoidance`) never
names the object either.

`sim_time_s` comes from Gazebo's `/stats` topic (gz Python bindings, host
machine only) and falls back to wall-clock seconds since the recorder started
when unavailable (e.g. run from a remote machine, see [Remote agents](remote.md)). `lat`/`lon` are
derived from the recorded ENU `pos_x`/`pos_y` about the reference point — the
same projection `waypoint_follower` uses in reverse.

**Mission/waypoint-tracking columns** (`mission_id` onward) only populate when
the *whole* scenario dict is available (always true for the host's own
`record_csv`; see the caveat in [Remote agents](remote.md) for the standalone/remote script). The
recorder reconstructs each agent's intended straight-line path — spawn →
mission 1's waypoints → mission 2's → ... — purely from the JSON, and compares
the *actual* simulated position against it every sample. It keys this by the
actual spawned instance name (`f"{id}{i}"`, the same naming
`agents_manager.py` uses for every agent on `/<world>/poses` — e.g. a
`nb_agents: 1` agent `"id": "bluerov1"` spawns as agent `bluerov10`, not the
bare `bluerov1`), not the raw scenario `"id"`, so this works for
multi-instance agent blocks too.

- `distance_to_target_m` / `cross_track_error_m` / `along_track_progress_pct`
  — straight-line distance to the current target waypoint, perpendicular
  drift off the *planned* line for the current leg (what a current or
  avoidance detour would cause), and progress along that line (0–100%,
  unaffected by drift).
- `waypoint_arrived` / `arrival_error_m` / `arrival_error_pct` — set on the
  sample where the agent comes within that mission's `range_tolerance` of the
  target (the same test `waypoint_follower` itself uses); `arrival_error_pct`
  expresses that miss distance as a percentage of the leg length, comparable
  across legs of different sizes. The recorder then auto-advances to the next
  waypoint/mission on its own — no coupling to the task's actual BT state.
  For the polyline guidance tasks (`waypoints` as `[[dx, dy, z], ...]`, dx
  north and dy east of the spawn, as the guidance reads them), the recorder
  also moves to the next segment once the vehicle passes the end of the
  current one, as the guidance does, even if it never came within the
  tolerance of the corner.
- `mission_complete` — `1` once the agent has arrived at the last waypoint of
  its last mission.

Every mission in an agent's list is a behaviour-tree root ticked on the same
timer, so they all run at once rather than in sequence. `mission_id` and
`task_type` therefore name the mission the tracking columns refer to when the
agent has a trackable one, and otherwise list all of the agent's concurrent
missions joined by `|` — a GNC pipeline row reads
`navigation|crossing_layers|control|allocation|metrics`.

Caveats: only `waypoint_follower` missions can be progress-tracked (they are
the only task type with an explicit, known target). The tracking pointer
walks those in order, advancing on arrival; missions of any other `task` are
never tracked and never block it. Waypoints within one mission are assumed
visited strictly in order, no looping (matches `"loop": false"`).

**`summary.csv`** — a second file (`<outdir>/<prefix>summary.csv`), one row
per agent, written once — either when the recorder detects that every
agent it's tracking a mission for has arrived (it stops recording itself
at that point: sampling timers cancelled, files closed, the rest of the
simulation — Gazebo, Unity, every agent — left running, since the world can
outlive this one batch of missions), or when the recorder shuts down with
the process, whichever comes first. A hard kill before either of those loses
it, unlike the per-sample files, which flush continuously. The agent's
`sensors`/`actuators` equipment manifest (`;`-joined, e.g. `"battery;sonar"`,
empty for an unequipped prop), objective (spawn → final target lat/lon),
outcome (`mission_complete`, final arrival error), difficulty indicators
(`max_cross_track_error_m`,
`min_sonar_distance_m`, `time_in_sonar_contact_s`, `distinct_obstacles_detected`
— a COUNT of separate obstacles that ever registered a contact, not which
ones; same no-identity rule as the per-sample sonar columns above),
battery start/end, run duration, and the world origin + `ocean_current`
repeated as global columns on every row. A compact numeric fact sheet for the
whole run — meant to be consumed by something else (e.g. an LLM writing a
narrative report), not itself prose.

**Scenario-wide completion log** — independent of `record_csv` entirely:
`simulation_run.mission_watcher.MissionWatcher` is always added to the host
executor, polls the live agent registry, and logs one line
(`All missions complete (N tracked) — scenario finished.`) once every
mission root has left `RUNNING`. A task that's meant to run forever opts out
via the `TaskAgent.PERPETUAL = True` class attribute so it never blocks this
from firing. Each agent already logs its own mission completion individually
(`Mission '<id>' finished with SUCCESS...`); this is the one line that says
the whole scenario is done, not just one vehicle.

To record from a machine other than the host (or independently of any
particular scenario launch), run the standalone CLI instead of setting
`record_csv` in the JSON:

```bash
python3 src/simulation_run/scripts/log_run_csv.py --world energy \
    --outdir csv_logs [--rate 2.0] [--prefix run1_]
```

It is the same `CsvRecorder`, minus mission tracking (it has no scenario JSON
to read a path from) and minus the `/stats` sim-time source unless run on the
host machine.

## Sensor streams: a ROS 2 bag

The CSV holds states. To keep the streams themselves (images, scans, IMU, navigation, ground truth on
`/<world>/poses`, wave components, wind), add one top-level key:

```jsonc
"record_bag": true                                        // every topic
"record_bag": ["/energy/poses", "/energy/wamv_xdyn0/imu"] // only these
```

The launcher records into `$LOG_DIR/bag` (log in `$LOG_DIR/rosbag.log`) and stops the bag cleanly at the
end of the run. Read it with `ros2 bag info`. Example: [`example_rosbag/record_one_topic.json`](../../src/simulation_run/config/example_rosbag/record_one_topic.json). Raw images fill a disk quickly: list the topics for long runs.
