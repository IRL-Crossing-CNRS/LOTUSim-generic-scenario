# LOTUSim Generic Scenario

**LOTUSim Generic Scenario** is a multi-agent simulation workspace built on top of the
[LOTUSim core](https://github.com/IRL-Crossing-CNRS/LOTUSim). It runs multi-domain
scenarios (surface, underwater, aerial) with:

- **ROS 2** (Jazzy on Ubuntu 24.04) for inter-process communication
- **Gazebo** for physics simulation and the 3D world
- **XDyn** for the hydrodynamic model (surface/underwater)
- **Unity** (optional) for high-fidelity 3D rendering, bridged over `ros_tcp_endpoint`

Agents are **mission-driven**: behaviour comes from a Behaviour Tree described
in the scenario JSON, not hard-coded per vehicle.

**This README is the user guide**: run a scenario and read the results
(installing is covered by [How_To_Install_LOTUSim](https://github.com/IRL-Crossing-CNRS/How_To_Install_LOTUSim)). It assumes you want to *use* the simulator, not modify it.
If you are going to change the code — add a vehicle, write a mission task,
plug in your own navigation/guidance/control algorithm — start instead at
[doc/DEVELOPER.md](doc/DEVELOPER.md).

---

## Before you start

Install first: everything about installing — nix, the two workspaces, the CPU
or GPU detection, PX4, the graphics bridge — is in
[How_To_Install_LOTUSim](https://github.com/IRL-Crossing-CNRS/How_To_Install_LOTUSim). Its script clones and builds both
workspaces; this guide starts once it has run.

After changing code in this workspace, rebuild it from its devshell:

```bash
lotusim_shell
colcon build --symlink-install
```

After changing the core, rebuild it first, from any terminal:

```bash
lotusim_core build      # lotusim_core rebuild to start from a clean tree
```

---

## Running a scenario

Scenario JSON files live in `src/simulation_run/config/`. A minimal agent
entry looks like:

```json
{
  "world_file": "energy.world",
  "agents": [
    {
      "id": "wp_follower",
      "class": "Bluerov2_heavy",
      "spawn": { "x": 0.0, "y": 0.0, "z": -10.0, "yaw": 0.0 },
      "tick_rate_hz": 1.0,
      "missions": [
        { "id": "patrol", "type": "action", "task": "waypoint_follower",
          "params": { "loop": true, "waypoints": [ { "lat": 50.32950, "lon": -4.19400 } ] } }
      ]
    }
  ],
  "renderer_unity": true
}
```

Waypoints are either inline as above, or in a patrol file:

```json
"params": { "waypoints_file": "../waypoints/waypoint_windturbine1.json" }
```

The guidance parameters (`guidance_mode`, `guidance_clock`, `control_rate_hz`,
`range_tolerance`, the velocity and acceleration limits) are optional and fall
back to the task defaults.

Launch it:

```bash
./src/simulation_run/executable/scenario_launch.sh --config my_scenario.json
```

`ROS_IP` is auto-detected (override by exporting it yourself before running,
e.g. `export ROS_IP=192.168.1.42`, if Unity needs a specific interface).
Add `--debug` for verbose output and `--gui` to show the Gazebo GUI
(every option and environment variable: [doc/LAUNCH_OPTIONS.md](doc/LAUNCH_OPTIONS.md)). The
helper processes run detached and write to their log file; `--terminals`
gives each one its own window instead. `--close-terminals` closes every GNOME terminal
window at the end of the run, your own included.

### Large fleets

Every agent is a Python node, and one Python process uses a single core. The
launcher therefore spreads the agents over several processes, sized from the
fleet, so that each control loop keeps its configured rate. `--shards N`
sets the number of processes yourself; `--shards 1` keeps every agent in one.
The processes share one world over ROS 2, so a scenario behaves the same
whatever the count.

Count about **0.07 CPU core per agent**. The launcher uses at most `nproc / 3`
processes, to leave cores to the world, the xdyn servers and PX4, and warns
when a fleet needs more: the world still runs in real time, but the control
loops run slower than configured. All the processes run on the local machine;
to run agents elsewhere, see [deployment/README.md](deployment/README.md).

**For the full JSON schema (every key, every built-in BT task and its
parameters, host vs. remote differences) see
[doc/scenario/](doc/scenario/README.md) — this section is only a
quick start.**

If Unity rendering is enabled (`renderer_unity: true`), launch the Unity
executable, enter the address the launcher prints ("Unity players connect
to", or under WSL "Unity player on this PC" / "on other machines") and ROS
port `10000`, and pick **Spectator
Mode** (free-fly, `W`/`A`/`S`/`D`/`Q`/`E` + mouse) or target-follower mode
(arrow keys to cycle agents).
On Ubuntu, log in with an **Ubuntu on Xorg** session (gear icon on the login
screen): under the default Wayland session the player runs at a lower frame
rate.

The Unity players are not in git. Each one is an asset of this repository's
release `v1.3`, pinned with its sha256 in
`lotusim_unity_executables/players.json`: the launcher downloads it into
`lotusim_unity_executables/<player>/` the first time, and again whenever
`players.json` changes. Under WSL, the Windows player goes in
`%LOCALAPPDATA%\LOTUSim\players\<player>\` instead: Windows cannot start a
Unity 6.6 player from the WSL filesystem (it fails to load `dstorage.dll`).
To run the simulation from Windows and connect players on other machines,
see [Running LOTUSim from Windows (WSL)](doc/WINDOWS_WSL.md).
The download goes
through the GitHub CLI: log in once with `gh auth login`. To fetch a player
without running a scenario:

```bash
./src/simulation_run/executable/fetch_unity_player.sh lotusimenergy_linux
```

The Linux and Windows players are **Unity 6.6** builds (`6000.6.6f1`, HDRP
17.6) of LOTUSim-Energy. When the scenario has a `"waves"` or
`"weather"` block, they draw on the ocean the waves the xdyn vessels ride,
2048 m around the camera (see [Waves and weather](doc/scenario/sea.md)).

### Scenario catalog

Scenario JSON files are grouped by subject in subdirectories of
`src/simulation_run/config/`. Each subdirectory has its own README listing
its files.

| Directory | Contents |
| --- | --- |
| `basic_examples/` | Behaviour-tree fundamentals on a single BlueROV: waypoint following, sequence and parallel nodes, a custom-task scaffold, accelerated world clocks, and an empty world |
| `current_examples/` | Ocean-current models (`ekman`, `gauss`, `copernicus`, `none`) with a BlueROV2 under PID, in station-keeping and transect form |
| `wind_wake_examples/` | Wind regions, turbine wake, and PX4 aerial drones |
| `wave_examples/` | One sea for every hull and the renderer: the weather of a date, unshared seas, a scheduled sea-state change |
| `multi_vehicle_examples/` | One scenario per vehicle class and mode (Kinematic-controllable or xdyn drift-only) |
| `example_rosbag/` | Recording a ROS 2 bag from the scenario file (`record_bag`) |
| `facet_demo/` | Full-fleet turbine-inspection demos: BlueROV, WAMV, and X500 agents across a 16-turbine farm |
| `iros_demo/` | The demo scenario: 59 agents (WAMV, BlueROV2, X500, LRAUV) working the 16-turbine farm, with wind and wake |
| `waypoints/` | Patrol waypoint files referenced by `waypoints_file`. These are data files, not scenarios |

Pass the path relative to the config directory:

```bash
./src/simulation_run/executable/scenario_launch.sh --config wind_wake_examples/wake_crossing_demo.json
```

The full index, including the per-vehicle capability table, is in
[src/simulation_run/config/README.md](src/simulation_run/config/README.md).

Turbine hub altitude differs between scenarios: 52 m in `facet_demo/` and in
`wind_wake_examples/wake_crossing_demo.json`, 85 m in
`wind_wake_examples/px4_offboard_patrol_test.json`. Within a single file,
`wake.turbines[].z` and the flight-path `z` values must match each other;
they are not shared across files.

---

## Leap Motion

A Leap Motion needs the Ultraleap tracking service. On Linux, download the `.deb` from
[Ultraleap Hand Tracking Software](https://www.ultraleap.com/downloads/) (the
Ultraleap APT repository is no longer online), install it, then unplug and
replug the camera:

```bash
sudo apt install ./tracking-software-linux-x64-6.2.0.deb
sudo leapctl eula
sudo systemctl enable --now ultraleap-hand-tracking-service
```

Red lights on the camera mean it is tracking.

---

## PX4 drones

X500 drones flown by PX4 SITL: installing PX4, enabling it on an agent, flying
by hand or offboard. See [doc/PX4_DRONES.md](doc/PX4_DRONES.md).

---

## Documentation

| Doc | Covers |
|---|---|
| [doc/DEVELOPER.md](doc/DEVELOPER.md) | **Start here to modify the code.** Where everything lives, the four extension points, frame/rebuild conventions, how to verify a change, known gaps |
| [doc/ARCHITECTURE.md](doc/ARCHITECTURE.md) | Repository/package organisation, agent class hierarchy, orchestration flow, global diagrams |
| [doc/MISSIONS.md](doc/MISSIONS.md) | The Behaviour Tree framework: engine, task lifecycle, built-in tasks, diagrams, references |
| [doc/scenario/](doc/scenario/README.md) | Generic scenario reference: every key, the behaviour trees, every task and its parameters, host vs. remote |
| [doc/LAUNCH_OPTIONS.md](doc/LAUNCH_OPTIONS.md) | Every launcher option and environment variable: debug, full-screen Unity, xdyn, another clone of the core, logs |
| [doc/ADD_A_VEHICLE.md](doc/ADD_A_VEHICLE.md) | Adding a vehicle end to end: Gazebo model, xdyn model, agent class, Unity prefab |
| [doc/WAMV_MODEL.md](doc/WAMV_MODEL.md) | How the WAM-V model was built and checked: parameter sources, failures and fixes, tests |
| [doc/PX4_DRONES.md](doc/PX4_DRONES.md) | X500 drones flown by PX4 SITL |
| [doc/LRAUV_PROPELLER_DEMO.md](doc/LRAUV_PROPELLER_DEMO.md) | Driving the LRAUV propeller from a ROS topic |
| [doc/ACCELERATED_SIMULATION.md](doc/ACCELERATED_SIMULATION.md) | Running a world faster than real time (real_time_factor, `guidance_clock`) |
| [doc/GNC_MODULAR_ARCHITECTURE.md](doc/GNC_MODULAR_ARCHITECTURE.md) | Navigation / Guidance / Control / Allocation as separate ROS2 nodes — topics, message schemas, what is implemented today, how to plug in your own algorithm for one block |
| [src/simulation_run/config/current_examples/](src/simulation_run/config/current_examples/) | BlueROV2 under PID across the four ocean-current models: the scenarios, what each current condition is, and how to run them |
| [deployment/README.md](deployment/README.md) | Running an agent from a remote machine (no Gazebo installed) |

---

## LRAUV propeller demo

Driving the LRAUV's propeller directly from a ROS topic, outside any mission:
[doc/LRAUV_PROPELLER_DEMO.md](doc/LRAUV_PROPELLER_DEMO.md).

---

## Video

A demonstrative video of LOTUSim is available on YouTube:

[![LOTUSim Video - IROS2026](https://img.youtube.com/vi/iXDz8ZqSpq4/0.jpg)](https://www.youtube.com/watch?v=iXDz8ZqSpq4)

## Relevant Publications

If you use [LOTUSim](https://github.com/naval-group/LOTUSim) in your research, or any of the repositories directly linked to LOTUSim

- [LOTUSim-Xdyn](https://github.com/naval-group/LOTUSim-Xdyn),
- [LOTUSim-generic-scenario](https://github.com/naval-group/LOTUSim-generic-scenario),
- [LOTUSim-Unity-modules](https://github.com/naval-group/LOTUSim-Unity-modules),
- [LOTUSim-UI-frontend](https://github.com/naval-group/LOTUSim-UI-frontend),
- [LOTUSim-UI-backend](https://github.com/naval-group/LOTUSim-UI-backend),

Please cite:

```bibtex
@inproceedings{LOTUSim26iros,
  title     = {{LOTUSim}: Multi-Domain Simulator for Marine Robotics},
  author    = {Buche, Cedric and Grosset, Juliette and Lechene, Helene and Dubromel, Marie and Havez-Bodivit, Pierig and Neo, Malcom and Prodhon, Julien},
  booktitle = {2026 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
  year      = {2026},
  publisher = {IEEE}
}
```
