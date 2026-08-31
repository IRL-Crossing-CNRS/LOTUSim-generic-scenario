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

**This README is the user guide**: install the workspace, run a scenario,
read the results. It assumes you want to *use* the simulator, not modify it.
If you are going to change the code — add a vehicle, write a mission task,
plug in your own navigation/guidance/control algorithm — start instead at
[doc/DEVELOPER.md](doc/DEVELOPER.md).

---

## Installation

### Option 1 — Automated (recommended)

```bash
chmod +x install_core_and_generic_scenario.sh
./install_core_and_generic_scenario.sh
```

Installs nix and its caches if absent, clones the LOTUSim core into
`~/lotusim_ws`, builds it through the core's own mise task inside the flake
environment, then clones and builds this workspace in that same environment.
Idempotent — safe to re-run.

### Option 2 — Manual

```bash
mkdir -p ~/Documents/workspace/lotusim && cd ~/Documents/workspace/lotusim
sudo apt update && sudo apt install -y jq
git clone --recurse-submodules https://github.com/IRL-Crossing-CNRS/LOTUSim-generic-scenario
cd LOTUSim-generic-scenario
git submodule update --init --remote --merge
```

### Build & source

The core provides its own ROS 2 and Gazebo through a nix flake, and builds with
mise. Both workspaces must be built inside that environment: a build against a
system ROS and one against the flake's produce different message definitions,
and the two never talk to each other.

```bash
# Core (at ~/lotusim_ws/src/LOTUSim)
cd $HOME/lotusim_ws/src/LOTUSim
nix develop            # ROS 2, Gazebo and xdyn, from the flake
mise run build

# This workspace, from inside that same shell
source $HOME/lotusim_ws/src/LOTUSim/install/setup.bash
cd $HOME/Documents/workspace/lotusim/LOTUSim-generic-scenario/
colcon build
source install/setup.bash
```

Nix itself is installed once per machine:

```bash
curl --proto '=https' --tlsv1.2 -L https://nixos.org/nix/install | sh -s -- --daemon
sudo tee -a /etc/nix/nix.conf <<'EOF'
experimental-features = nix-command flakes
extra-substituters = https://ros.cachix.org
extra-trusted-public-keys = ros.cachix.org-1:dSyZxI8geDCJrwgvCOHDoAfOm5sV1wCPjBkKL+38Rvo=
EOF
sudo systemctl restart nix-daemon
```

The ROS cache is not optional in practice: without it nix rebuilds the whole
ROS 2 stack from source.

**PX4 SITL stays a system build**, and the launcher keeps it that way: inside a
devshell it is handed the system loader, binary and gz configuration paths, so
its gz dependencies resolve against the ROS 2 install it was linked with. The
two builds still meet over gz-transport. Build it with the same compiler as
before (`CC=clang CXX=clang++ make px4_sitl_default`) — the flags PX4 passes are
clang's.

**Rendering sensors need a GPU bridge** away from NixOS. A camera or gpu_lidar
is rendered whether or not a window is open, and a nix-built Gazebo cannot reach
the host driver by itself:

```bash
nix profile add github:nix-community/nixGL#nixGLIntel
```

On a hybrid or NVIDIA machine, install the matching NVIDIA wrapper instead and
name it with `LOTUSIM_GL_WRAPPER`, since the core's detection prefers Intel:

```bash
NIXPKGS_ALLOW_UNFREE=1 nix profile add --impure github:nix-community/nixGL#nixGLNvidia
export LOTUSIM_GL_WRAPPER=nixGLNvidia-<driver version>
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
Add `--debug` for verbose output, `--gui` to show the Gazebo GUI.

**For the full JSON schema (every key, every built-in BT task and its
parameters, host vs. remote differences) see
[doc/WRITE_SCENARIO.md](doc/WRITE_SCENARIO.md) — this section is only a
quick start.**

If Unity rendering is enabled (`renderer_unity: true`), launch the Unity
executable, enter your local IP and ROS port `10000`, and pick **Spectator
Mode** (free-fly, `W`/`A`/`S`/`D`/`Q`/`E` + mouse) or target-follower mode
(arrow keys to cycle agents).

### Scenario catalog

Scenario JSON files are grouped by subject in subdirectories of
`src/simulation_run/config/`. Each subdirectory has its own README listing
its files.

| Directory | Contents |
| --- | --- |
| `basic_examples/` | Behaviour-tree fundamentals on a single BlueROV: waypoint following, sequence and parallel nodes, a custom-task scaffold, accelerated world clocks, and an empty world |
| `current_examples/` | Ocean-current models (`ekman`, `gauss`, `copernicus`, `none`) with a BlueROV2 under PID, in station-keeping and transect form |
| `wind_wake_examples/` | Wind regions, turbine wake, and PX4 aerial drones |
| `multi_vehicle_examples/` | One scenario per vehicle class and mode (Kinematic-controllable or xdyn drift-only) |
| `facet_demo/` | Full-fleet turbine-inspection demos: BlueROV, WAMV, and X500 agents across a 16-turbine farm |
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

## PX4 SITL (aerial drones)

An `X500` agent with `"px4": true` in its scenario JSON is flown by an
external **PX4 SITL** process instead of a built-in BT controller — PX4
attaches to the airframe LOTUSim spawns in the aerial world and reads its
Gazebo sensors / writes its motor commands directly
(see [doc/ARCHITECTURE.md §5.1](doc/ARCHITECTURE.md)). PX4 itself is not part
of this repo: it's a separate checkout you build once.

### 1. Clone and build PX4-Autopilot

```bash
git clone --recursive https://github.com/PX4/PX4-Autopilot.git ~/PX4-Autopilot
cd ~/PX4-Autopilot
```

System packages needed beyond a base Ubuntu 24.04 dev machine (all via
`sudo apt install`, no PPAs needed):

```bash
sudo apt install -y ninja-build ccache libopencv-dev \
    libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev
```

Python packages (PX4's `Tools/setup/requirements.txt`; `--break-system-packages`
is required on Ubuntu 24.04's PEP 668-managed Python, and this only touches
your user site-packages, not the system Python):

```bash
pip3 install --user --break-system-packages -r Tools/setup/requirements.txt
```

**Known build issue (clang 18+):** a few PX4 source files use
variable-length arrays in C++, which clang 18 treats as a hard error
(`-Werror` + `-Wvla-cxx-extension`) even though older clang only warned.
If `make` fails with `variable length arrays in C++ are a Clang extension`,
add one line to `cmake/px4_add_common_flags.cmake` in the clang branch
(next to the existing `-Wno-c99-designator` etc.):

```cmake
-Wno-error=vla-cxx-extension
```

Then build the Gazebo x500 target:

```bash
make px4_sitl gz_x500
```

This command builds PX4 and then launches it in an interactive `pxh>` shell
attached to whatever Gazebo world and model it finds. With no matching Gazebo
instance running, PX4 attaches to nothing and the `pxh>` prompt redraws in a
loop indefinitely. If the run is backgrounded or redirected to a log file,
that file grows without bound (several GB within minutes) until the disk
fills. After the first build, check for a leftover process with
`ps aux | grep bin/px4`, kill it, and delete the log. Later scenario launches
are unaffected: LOTUSim starts its own PX4 instance with `-d` (daemon mode,
no `pxh>` prompt).

### 2. Point LOTUSim at your PX4 checkout

`X500._start_px4_sitl()` (in `lotusim_sdk/agents/entity/physical/x500.py`)
looks for a built PX4 checkout at `$PX4_AUTOPILOT_PATH`, defaulting to
`~/PX4-Autopilot`. If you cloned it elsewhere, export the real path before
launching:

```bash
export PX4_AUTOPILOT_PATH=/path/to/PX4-Autopilot
```

### 3. Enable PX4 on an X500 agent

In the scenario JSON:

```json
{
  "id": "x500_px4",
  "class": "X500",
  "spawn": { "x": 20, "y": 30, "z": 0, "yaw": 0.0 },
  "px4": true,
  "px4_control": "manual",
  "sdf_file": "model.sdf",
  "xdyn": false
}
```

On a `"px4": true` agent, `spawn.z` must be `0`. An unarmed multirotor
produces no thrust, so a spawn above the surface falls under gravity before
PX4 finishes booting. The resulting impact corrupts EKF2's attitude and
velocity estimates, which surfaces as a persistent "not ready to arm" in
QGroundControl. To start a mission at altitude, climb after arming using
`takeoff_alt_m` in `px4_offboard_patrol`'s params (below) instead of raising
`spawn.z`. Agents without `"px4": true` are unaffected and may spawn at any
altitude.

Then launch the scenario as usual — PX4 SITL starts automatically once the
agent's spawn is confirmed, one instance per PX4-enabled agent:

```bash
./src/simulation_run/executable/scenario_launch.sh --config facet_demo/demo_facet_physics.json
```

Per-instance PX4 console output lands in
`scenario_logs/<timestamp>/px4_sitl_<agent_name>.log` — check there first if
a PX4 drone isn't responding. A clean attach looks like `gazebo already
running world: aerialWorld` / `PX4_GZ_MODEL_NAME set, PX4 will attach to
existing model`, followed by MAVLink listening on UDP port 14550.

### 4. Flying it: manual or offboard

`px4_control: "manual"` — PX4 takes no commands from LOTUSim's BT framework.
The drone is flown over MAVLink from QGroundControl (next subsection).
Example: `wind_wake_examples/px4_manual_wake_flying.json`.

`px4_control: "offboard"` — a `px4_offboard_patrol` mission task arms the
vehicle, takes off, switches to OFFBOARD, and streams position setpoints
through a waypoint list over MAVLink. No ground control station is required.
Examples: `wind_wake_examples/wake_crossing_demo.json` and
`wind_wake_examples/px4_offboard_patrol_test.json`. Parameters, under the
task's `missions[].params`:

| Param | Default | Meaning |
|---|---|---|
| `spawn` | required | This agent's own `{x, y, z}` spawn. MAVLink's local NED frame is zeroed there, so waypoints are converted from world coordinates relative to it. |
| `waypoints` | required | `[{"name", "x", "y", "z"}, ...]` in the same world frame as `spawn`. `name` is optional. |
| `loop` | `true` | Cycle back to the first waypoint after the last, instead of finishing. |
| `hold_radius_m` | `5.0` | 3D distance to a waypoint that counts as "reached". |
| `takeoff_alt_m` | `15.0` | Climb straight up to this altitude (above spawn) before heading to the first waypoint. |
| `setpoint_rate_hz` | `10.0` | Offboard setpoint stream rate. PX4 leaves OFFBOARD mode if the stream drops below 2 Hz. |

`px4_control` is descriptive only; the attached task is what drives the
vehicle. Keep the two consistent, as setting `"manual"` while attaching
`px4_offboard_patrol` (or the reverse) has no defined behaviour.

#### QGroundControl

QGroundControl is not in Ubuntu's apt repositories. Download the official
AppImage:

```bash
curl -L -o ~/Applications/QGroundControl-x86_64.AppImage \
  "$(curl -s https://api.github.com/repos/mavlink/qgroundcontrol/releases/latest \
     | grep browser_download_url | grep x86_64.AppImage | cut -d'"' -f4)"
chmod +x ~/Applications/QGroundControl-x86_64.AppImage
~/Applications/QGroundControl-x86_64.AppImage
```

QGroundControl connects automatically to any PX4 SITL instance listening on
`localhost:14550`. With the scenario running it detects the vehicle within a
few seconds and shows live telemetry. Arm and command takeoff or goto from
its flight controls.

---

## Documentation

| Doc | Covers |
|---|---|
| [doc/DEVELOPER.md](doc/DEVELOPER.md) | **Start here to modify the code.** Where everything lives, the four extension points, frame/rebuild conventions, how to verify a change, known gaps |
| [doc/ARCHITECTURE.md](doc/ARCHITECTURE.md) | Repository/package organisation, agent class hierarchy, orchestration flow, global diagrams |
| [doc/MISSIONS.md](doc/MISSIONS.md) | The Behaviour Tree framework: engine, task lifecycle, built-in tasks, diagrams, references |
| [doc/WRITE_SCENARIO.md](doc/WRITE_SCENARIO.md) | Full scenario JSON reference — every parameter, host vs. remote |
| [doc/ACCELERATED_SIMULATION.md](doc/ACCELERATED_SIMULATION.md) | Running a world faster than real time (real_time_factor, `guidance_clock`) |
| [doc/GNC_MODULAR_ARCHITECTURE.md](doc/GNC_MODULAR_ARCHITECTURE.md) | Navigation / Guidance / Control / Allocation as separate ROS2 nodes — topics, message schemas, what is implemented today, how to plug in your own algorithm for one block |
| [src/simulation_run/config/current_examples/](src/simulation_run/config/current_examples/) | BlueROV2 under PID across the four ocean-current models: the scenarios, what each current condition is, and how to run them |
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
