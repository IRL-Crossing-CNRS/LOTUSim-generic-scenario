# LOTUSim Generic Scenario

**LOTUSim Generic Scenario** is a multi-agent simulation workspace built on top of the
[LOTUSim core](https://github.com/IRL-Crossing-CNRS/LOTUSim). It runs multi-domain
scenarios (surface, underwater, aerial) with:

- **ROS 2** (Jazzy on Ubuntu 24.04) for inter-process communication
- **Gazebo** for physics simulation and the 3D world
- **XDyn** for the hydrodynamic model (surface/underwater)
- **Unity** (optional) for high-fidelity 3D rendering, bridged over `ros_tcp_endpoint`

Agents are **mission-driven**: behaviour comes from a Behaviour Tree described
in the scenario JSON, not hard-coded per vehicle. See
[Documentation](#documentation) below for the full picture.

---

## Installation

### Option 1 — Automated (recommended)

```bash
chmod +x install_core_and_generic_scenario.sh
./install_core_and_generic_scenario.sh
```

Checks that you run Ubuntu 24.04 (ROS 2 Jazzy), installs system/Python
dependencies, clones the LOTUSim core into `~/lotusim_ws`, configures
`~/.bashrc`, builds the core workspace, and clones/builds this workspace.
Idempotent — safe to re-run. If ROS 2 is already installed, the ROS 2 apt
repository and `lotusim install` steps are skipped (force the latter with
`FORCE_LOTUSIM_INSTALL=1`).

### Option 2 — Manual

```bash
mkdir -p ~/Documents/workspace/lotusim && cd ~/Documents/workspace/lotusim
sudo apt update && sudo apt install -y jq
git clone --recurse-submodules https://github.com/IRL-Crossing-CNRS/LOTUSim-generic-scenario
cd LOTUSim-generic-scenario
git submodule update --init --remote --merge
```

### Build & source

```bash
# Core (at ~/lotusim_ws/src/LOTUSim) — use lotusim clean_build, not colcon build
source /opt/ros/jazzy/setup.bash
lotusim clean_build
source $HOME/lotusim_ws/install/setup.bash

# This workspace
cd $HOME/Documents/workspace/lotusim/LOTUSim-generic-scenario/
colcon build
source install/setup.bash
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
  "aerial_domain": false,
  "renderer_unity": true
}
```

Launch it:

```bash
./src/simulation_run/executable/scenario_launch.sh --config my_scenario.json
```

`ROS_IP` is auto-detected (override by exporting it yourself before running,
e.g. `export ROS_IP=192.168.1.42`, if Unity needs a specific interface).
Add `--debug` for verbose output, `--gui` to show the Gazebo GUI.

**For the full JSON schema (every key, every built-in BT task and its
parameters, host vs. remote differences) see
[doc/WRITE_SCENARIO.md](doc/WRITE_SCENARIO.md) — this section is only a
quick start.**

If Unity rendering is enabled (`renderer_unity: true`), launch the Unity
executable, enter your local IP and ROS port `10000`, and pick **Spectator
Mode** (free-fly, `W`/`A`/`S`/`D`/`Q`/`E` + mouse) or target-follower mode
(arrow keys to cycle agents).

---

## Documentation

| Doc | Covers |
|---|---|
| [doc/ARCHITECTURE.md](doc/ARCHITECTURE.md) | Repository/package organisation, agent class hierarchy, orchestration flow, global diagrams |
| [doc/MISSIONS.md](doc/MISSIONS.md) | The Behaviour Tree framework: engine, task lifecycle, built-in tasks, diagrams, references |
| [doc/WRITE_SCENARIO.md](doc/WRITE_SCENARIO.md) | Full scenario JSON reference — every parameter, host vs. remote |
| [deployment/README.md](deployment/README.md) | Running an agent from a remote machine (no Gazebo installed) |

---

## LRAUV propeller demo

The propeller has currently only been developed for the LRAUV, as a
standalone (non-BT) demo of direct thruster control. Spawn an
`Lrauv_Propeller` agent in your scenario JSON (same `agents` entry shape as
above, no `missions` needed):

```json
{
  "id": "lrauvpropeller",
  "class": "Lrauv_Propeller",
  "spawn": { "x": 0.0, "y": 0.0, "z": -100.0 }
}
```

### Manual control via ROS topic

With the scenario running, open a second terminal, source the workspaces
(`/opt/ros/<distro>/setup.bash`, `~/lotusim_ws/install/setup.bash`, this
workspace's `install/setup.bash`), then:

```bash
# Start the propeller RPM sequence
ros2 topic pub /defenseScenario/lrauvpropeller0/control_lrauv std_msgs/msg/Bool "data: true"

# Stop it (also sends one command at rpm=100.0, pd=0.88)
ros2 topic pub /defenseScenario/lrauvpropeller0/control_lrauv std_msgs/msg/Bool "data: false"
```

(Replace `defenseScenario` with your scenario's world name.) Tune the RPM
values sent while running in
`src/external_packages/lrauv_propeller/lrauv_propeller/lrauv_propeller.py`,
in `send_propeller_command(rpm=..., pd=...)`.

### Auto-start cycle

To make the agent automatically cycle between its `propeller_phases` (high
RPM / low RPM) on spawn instead of waiting for a manual `control_lrauv`
command, uncomment the `self.start_sequence()` call in that file's
`__init__`. Rebuild (`colcon build`) and re-source after any change.

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
- [LOTUSim-UI-frontend](https://github.com/naval-group/LOTUSim-UI-backend),

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
