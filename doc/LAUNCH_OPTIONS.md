# Launch options of the generic scenario

Every option of `scenario_launch.sh`, the launcher of the generic scenario
workspace, every environment variable it and the simulator read, and where
each run writes its logs. For a first run, the
[README](../README.md#running-a-scenario) is enough.

These options apply to generic scenarios only. The core's own commands
(`lotusim_core sim <world>`, `mise run sim <world>`) take only `--gui`,
`--debug` and `--assets-path`; among the variables below, only
`LOTUSIM_LOG_LEVEL` and `XDYN_ASSETS_PATH` apply to them.

```bash
./src/simulation_run/executable/scenario_launch.sh --config my_scenario.json [options]
```

`--config` takes a path, or a file name relative to
`src/simulation_run/config/` (`--config current_examples/transect_gauss.json`).

## Command-line options

| Option | Effect |
|---|---|
| `--config <file>` | Scenario JSON to run. Required |
| `--debug` | Verbose output everywhere: the simulator's plugin logs at `debug`, Gazebo at `-v4`, the Python agents at `DEBUG`, the ROS-Unity bridge at `DEBUG` |
| `--gui` | Opens the Gazebo GUI next to the server |
| `--terminals` | Runs each helper process (xdyn servers, ROS-Unity bridge, Gazebo worlds) in its own GNOME terminal window instead of detached; useful to watch one live |
| `--close-terminals` | Closes every GNOME terminal window at the end of the run, your own included |
| `--shards <N\|auto>` | Number of processes the agents are split across. `auto` (default) sizes it from the fleet; `1` keeps every agent in one process |

## Environment variables

Set them on the command line for one run, or export them in the shell:

```bash
UNITY_EXTRA_ARGS="-screen-fullscreen 1" ./src/simulation_run/executable/scenario_launch.sh --config my_scenario.json
```

### Unity

| Variable | Default | Effect |
|---|---|---|
| `UNITY_MODE` | `exe` | `exe` starts the Unity player. `editor` starts nothing: open the Unity project in the Editor and press Play |
| `UNITY_EXTRA_ARGS` | `-screen-fullscreen 0 -screen-width 1280 -screen-height 720` | Arguments passed to the Unity player. Full screen: `-screen-fullscreen 1`. Another window size: change `-screen-width` / `-screen-height`. Driver workarounds: `-force-glcore`, `-force-gfx-direct` |
| `UNITY_EXE_DIR` | `lotusim_unity_executables/<player>/`, under WSL `%LOCALAPPDATA%\LOTUSim\players\<player>\` | Runs another Unity player build, e.g. one being tested: `UNITY_EXE_DIR=~/builds/lotusimenergy_linux`. Under WSL, a Windows build must be on a Windows drive: `UNITY_EXE_DIR=/mnt/c/builds/lotusimenergy_windows` |

### xdyn

| Variable | Default | Effect |
|---|---|---|
| `XDYN_PATH` | unset | Directory holding `xdyn-for-cs`, when it is not already on `PATH` (outside the core's `nix develop` shell, or to run another xdyn build) |
| `XDYN_VERBOSE` | unset | `1` logs every xdyn message and websocket event to `scenario_logs/<date>/xdyn_<agent>.log`. Very chatty |
| `XDYN_ASSETS_PATH` | `$LOTUSIM_MODELS_PATH` | Root against which xdyn resolves the relative mesh and seabed paths of a model YAML |
| `BLUEROV_CURRENT` | the scenario's `bluerov_current`, else `ekman` | Ocean-current model of the `Bluerov2_heavy_current` agents, for a one-off override. The normal place is the scenario JSON |

### Other repositories and installs

The launcher finds the core simulator and this workspace from these
variables, so another clone or another branch runs without moving anything:

| Variable | Default | Effect |
|---|---|---|
| `LOTUSIM_WS` | `~/lotusim_ws` | Colcon workspace of the core simulator |
| `LOTUSIM_PATH` | `$LOTUSIM_WS/src/LOTUSim` | Core simulator clone: worlds, models, plugins. When it holds a `mise.toml` and `mise` is installed, the worlds start with `mise run sim` from that clone, otherwise with `lotusim run` |
| `LOTUSIM_SCENARIO_WS` | `~/Documents/workspace/lotusim/LOTUSim-generic-scenario` | This workspace: the scenario configs and the Unity players are read from there, not from the directory you launch from |
| `PX4_AUTOPILOT_PATH` | `~/PX4-Autopilot` | PX4 checkout, for scenarios with PX4 drones |

For example, to run a second clone of the core:

```bash
LOTUSIM_PATH=~/worktrees/LOTUSim-feature ./src/simulation_run/executable/scenario_launch.sh --config my_scenario.json
```

### Weather

The weather is chosen in the scenario, not here: a `"weather"` block with a
`date` and a `place` replays the waves, current and wind measured there and
then ([Waves and weather](scenario/sea.md)).
The first run of a place and date downloads them from the Copernicus Marine
Service with the `copernicusmarine` tool (`pip install copernicusmarine`,
then `copernicusmarine login` once); later runs read the saved copy, offline
and without an account. Neither variable is needed in the usual case:

| Variable | Default | Effect |
|---|---|---|
| `LOTUSIM_WEATHER_CACHE` | `~/.cache/lotusim/weather` | Folder of the saved copies, one JSON file per kind (waves, current, wind), place and date. Delete a file to download it again; copy the folder to another machine to run there without an account |
| `LOTUSIM_COPERNICUS_PYTHON` | `/usr/bin/python3` | Python that `copernicusmarine` was installed with. Change it only when you installed the tool in another Python (a venv, conda): the simulation's own Python environment would otherwise hide the tool's packages |

### Network

| Variable | Default | Effect |
|---|---|---|
| `ROS_IP` | first address of `hostname -I`, else `127.0.0.1` | Address Unity connects to. Set it when the machine has several interfaces |
| `ROS_DOMAIN_ID` | `0` | ROS 2 domain of every process of the run |

### PX4

| Variable | Default | Effect |
|---|---|---|
| `PX4_CPU_AFFINITY` | the last two cores (machines with 4 cores or more) | Cores PX4 and the aerial world are pinned to. A core list such as `6,7`, or empty to disable pinning |
| `PX4_SITL_LOG` | `scenario_logs/<date>/px4_sitl_<agent>.log` | PX4 SITL log file |

### Simulator logs

| Variable | Default | Effect |
|---|---|---|
| `LOTUSIM_LOG_LEVEL` | `info` | Level of the simulator's plugin logs: `trace`, `debug`, `info`, `warn`, `error`, `critical` or `off`. `--debug` sets `debug` |

## Logs

A run writes in two places, and neither is ever cleaned up:

| Where | What |
|---|---|
| `scenario_logs/<date>/` in the directory you launch from | The launcher's own output (`main_simulation.log`), the agents, the xdyn servers, the ROS-Unity bridge, Gazebo, PX4 |
| `$LOTUSIM_PATH/lotus_logs/<date>/` | The simulator's plugins, one file per plugin |

On a demo machine, or for long batches, run with `LOTUSIM_LOG_LEVEL=off`
so the plugins write nothing, and clear both directories from time to time.
