# LOTUSim Generic Scenario

## Overview

LOTUSim Generic Scenario is a multi-agent simulation framework built on ROS 2 and Gazebo.
This repository provides a ready-to-use scenario workspace on top of the [LOTUSim core](https://github.com/naval-group/LOTUSim).

Supports:
- Ubuntu 22.04 → ROS 2 Humble
- Ubuntu 24.04 → ROS 2 Jazzy

---

## Installation 

### Option 1 — Automated (Recommended)

Use the provided install script to set up everything in one step:

```bash
chmod +x install_core_and_generic_scenario.sh
./install_core_and_generic_scenario.sh
```

The script will automatically:
1. Detect your Ubuntu version and select the matching ROS 2 distribution
2. Install system dependencies (`git`, `jq`, `curl`, etc.)
3. Install Python dependencies (`pyarrow`, `pandas`, `matplotlib`, `opencv-python`)
4. Clone the LOTUSim core repository into `~/lotusim_ws`
5. Configure your `~/.bashrc` with all required environment variables
6. Run `lotusim install` to set up ROS 2 and Gazebo
7. Clone and build the LOTUSim Generic Scenario workspace

#### What Gets Installed

| Component | Location |
|---|---|
| LOTUSim Core | `~/lotusim_ws/src/LOTUSim` |
| Generic Scenario | `~/Documents/workspace/lotusim/LOTUSim-generic-scenario` |
| ROS 2 (Humble or Jazzy) | `/opt/ros/<distro>` |

The script is **idempotent** — safe to run multiple times. It will skip cloning if repositories already exist and skip `~/.bashrc` modifications if the LOTUSim block is already present.

---

### Option 2 — Manual

Clone the repository:

```bash
mkdir -p ~/Documents/workspace/lotusim
cd ~/Documents/workspace/lotusim
sudo apt update && sudo apt install -y jq
git clone --recurse-submodules https://github.com/naval-group/LOTUSim-generic-scenario
cd LOTUSim-generic-scenario
git submodule update --remote --merge
```

## Setup and Source the Packages

The architecture of the repositories [LOTUSim](https://github.com/naval-group/LOTUSim) and [Generic Scenario](https://github.com/naval-group/LOTUSim-generic-scenario) is defined in:  
`LOTUSim-generic-scenario/src/simulation_run/executable/scenario_launch.sh`

```sh
# -------------------- Paths --------------------
PATH=$HOME/lotusim_ws/src/lotusim/physics:$HOME/lotusim_ws/src/LOTUSim/launch:$PATH
LOTUSIM_WS=$HOME/lotusim_ws
LOTUSIM_PATH=$LOTUSIM_WS/src/LOTUSim
LD_LIBRARY_PATH=$LOTUSIM_PATH/physics:$LD_LIBRARY_PATH
LOTUSIM_MODELS_PATH=$LOTUSIM_PATH/assets/models

# --- Updated paths for your scenario workspace ---
LOTUSIM_SCENARIO_WS=$HOME/Documents/workspace/lotusim/LOTUSim-generic-scenario
CONFIG_DIR="$LOTUSIM_SCENARIO_WS/src/simulation_run/config"
UNITY_EXE_PATH="$LOTUSIM_SCENARIO_WS/lotusim_unity_executables/lotusim_scenario_linux/lotusim_scenario.x86_64"
```

Open a terminal, do these commands to build and source:

```sh
# Build and Source
source /opt/ros/humble/setup.bash
lotusim clean_build
source $HOME/lotusim_ws/install/setup.bash
```

```sh
cd $HOME/Documents/workspace/lotusim/LOTUSim-generic-scenario/
colcon build
source install/setup.bash 
```

Your setup is now complete, you’re ready to start a simulation!

## Configure the parameters for your simulation

All configuration files are located in: `/LOTUSim-generic-scenario/src/simulation_run/config/files.json`



### Spawning Agents: Supported Initialization Pose Formats

When spawning an agent in the simulation, the user can initialize its position in **two different ways**, depending on the number of elements provided in the pose array.

There are two accepted formats:

---

**1. Geographic Position (GeoPoint)**
Use this format when providing **latitude and longitude**, with an optional altitude.

Supported inputs:
- `[lat, lon]`
- `[lat, lon, alt]`

When one of these formats is detected, the system automatically sends a **GeoPoint** message.

---

**2. Full Pose (vessel_position)**
Use this format when providing a full **6-element pose**, including position and orientation:

- `[x, y, z, roll, pitch, yaw]`

When 6 elements are detected, the system sends a **vessel_position** message to place the agent at the specified Cartesian pose.

**Example (2-element GeoPoint):**
```json
"Wamv": {
  "nb_agents": 1,
  "poses": [
    [-34.8852, 138.6217]
  ],
  "model": "model.sdf",
  "xdyn": true
}
```

> **Note**: To enable the physics of the environment (waves and currents) set `"xdyn": true`


Below is complete example of a configuration file: `defenseScenario.json`

```json
{
  "world_file": "defenseScenario.world",

  "agents": {
    "Lrauv": {
      "nb_agents": 5,
      "poses": [
        [-2513.0, -2997.0, -100.0, 0.0, 0.0, 0.0],
        [-2515.0, -3000.0, -100.0, 0.0, 0.0, 0.0],
        [-2513.0, -3003.0, -100.0, 0.0, 0.0, 0.0],
        [-2518.0, -3003.0, -100.0, 0.0, 0.0, 0.0],
        [-2518.0, -2997.0, -100.0, 0.0, 0.0, 0.0]
      ],
      "model": "model.sdf",
      "xdyn": true
    },

    "Bluerov2_heavy": {
      "nb_agents": 1,
      "poses": [
        [-3000.0, -2500.0, -50.0, 0.0, 0.0, 0.0]
      ],
      "model": "model.sdf",
      "xdyn": true
    },

    "Mine": {
      "nb_agents": 1,
      "poses": [
        [0.0, 0.0, -100.0, 0.0, 0.0, 0.0]
      ],
      "model": "model.sdf",
      "xdyn": true
    },

    "Fremm": {
      "nb_agents": 1,
      "poses": [
        [-400.0, -4800.0, 0.0, 0.0, 0.0, 0.0]
      ],
      "model": "model.sdf",
      "xdyn": true
    },

    "Commando": {
      "nb_agents": 1,
      "poses": [
        [-2000.0, -2000.0, 0.0, 0.0, 0.0, 0.0]
      ],
      "model": "model.sdf",
      "xdyn": true
    },

    "Dtmb_hull": {
      "nb_agents": 1,
      "poses": [
        [-2000.0, -1500.0, 0.0, 0.0, 0.0, 0.0]
      ],
      "model": "model.sdf",
      "xdyn": true
    },
  },

  "aerial_domain": true,
  "renderer_unity": true
}
```

>**Note:** The `Dtmb_hull` agent does not have a corresponding 3D model in Unity and therefore cannot be rendered in Unity.

## Launch the Simulation

Once your configuration is ready, you can launch the simulation:

1. **Set your IP** — Update `ROS_IP`(to know your local IP address:`hostname -I`) in  
   `LOTUSim-generic-scenario/src/simulation_run/executable/scenario_launch.sh`

2. **Launch the Unity executable** — Located at:  
   `$HOME/Documents/workspace/lotusim/LOTUSim-generic-scenario/src/linux_executable/lotusim_scenario.x86_64`  
   _(Make sure the path matches the one defined in `scenario_launch.sh`.)_

3. **In the Unity window**, enter:  
   - Your local **IP address**  
   - **ROS port:** `10000`  
   - Choose **Spectator Mode** (free-fly camera), or leave it unchecked to follow entities (navigate with arrow keys)

4. **In the first terminal**, run:

    ```sh
    ./src/simulation_run/executable/scenario_launch.sh --config $HOME/Documents//workspace/lotusim/LOTUSim-generic-scenario/src/simulation_run/config/defenseScenario.json
    ```

    For debugging, append `--debug`:

    ```sh
    ./src/simulation_run/executable/scenario_launch.sh --config $HOME/Documents/workspace/lotusim/LOTUSim-generic-scenario/src/simulation_run/config/defenseScenario.json --debug
    ```

5. **In the second terminal**, source workspaces and launch the following bridge to activate these features:

    ```sh
    # Terminal: Source Workspaces
    source /opt/ros/humble/setup.bash
    source $HOME/lotusim_ws/install/setup.bash
    source $HOME/Documents/workspace/lotusim/LOTUSim-generic-scenario/install/setup.bash
    ```

    This bridge enables communication between Gazebo and ROS 2 for simulation telemetry and environmental effects.

    **Display Only Simulation Statistics (RTF)**

    To bridge only the simulation statistics (e.g., simulation time, Real-Time Factor):

    ```sh
    ros2 run gz_ros2_bridge stats_gz_to_ros_bridge
    ```

    **Add Wind**
    To bridge wind Gazebo to ROS 2:

    ```sh
    ros2 run gz_ros2_bridge wind_ros_to_gz_bridge
    ```

    **Launch Both Bridges Together**
    To start both bridge nodes simultaneously:

    ```sh
    ros2 launch gz_ros2_bridge bridge_nodes.launch.py
    ```

    This launch file starts the following bridges:

    - `wind_ros_to_gz_bridge` → Transfers wind and environmental data
    - `stats_gz_to_ros_bridge` → Transfers simulation statistics such as simulation time and RTF

6. **Terminals for the physics** will open automatically and the simulation will start

>When the connection LOTUSim-Unity is done, in the Unity window, at the top left corner, the arrows will go from red to blue

## Navigate in the Unity Scene

- **Spectator Mode**: Move using `W`, `A`, `S`, `D`, `Q`, `E`, and the mouse — or use **Leap Motion** hand tracking (see README/Wiki of the Unity repo).

- **Target Follower Mode**: The camera automatically cycles through agents.  
  - Use the **arrow keys** to switch between entities.

- **Change Wind Direction** (keyboard shortcuts):  
  - X-axis: `1` / `2`  
  - Y-axis: `4` / `5`  
  - Z-axis: `7` / `8`


## Use propeller/thrusters

Currently, the propeller have been developped only on the LRAUV.

To use an LRAUV with active propellers, initialise it in the config file `defenseScenario.json` by adding this block:

```json
"Lrauv_Propeller": {
  "nb_agents": 1,
  "poses": [
    [-2513.0, -2997.0, -100.0, 0.0, 0.0, 0.0]
  ],
  "model": "model.sdf",
  "xdyn": true
  }
```

---

### Manual control via ROS topic 

Start the simulation as usual with:
```sh
./src/simulation_run/executable/scenario_launch.sh --config $HOME/Documents/workspace/lotusim/LOTUSim-generic-scenario/src/simulation_run/config/defenseScenario.json
```

Then open a second terminal:

  ```sh
  # Terminal: Source Workspaces
  source /opt/ros/humble/setup.bash
  source $HOME/lotusim_ws/install/setup.bash
  source $HOME/Documents/workspace/lotusim/LOTUSim-generic-scenario/install/setup.bash
  ```
  
Whenever you are ready to send a propeller command, send in the second terminal:
  ```sh
  ros2 topic pub /defenseScenario/lrauvpropeller0/control_lrauv std_msgs/msg/Bool "data:  true"
  ```

You can change the value of the `rpm` sent by modifying the file `$HOME/Documents/workspace/lotusim/lotusim-generic-scenario/src/external_packages/lrauv_propeller/lrauv_propeller/lrauv_propeller.py`, in this line:
  ```py
  self.send_propeller_command(rpm=100.0, pd=0.88) # example to send a propeller command
  ```

This command will send the defaut high rpm value set in `lrauv_propeller.py`
  ```sh
  ros2 topic pub /defenseScenario/lrauvpropeller0/control_lrauv std_msgs/msg/Bool "data:  false"
  ```

---
### Propeller with auto-start cycle

Finally, to use the propeller with a cycle between high and low values, simply uncomment the third line in `lrauv_propeller.py`:
```py
# Automatically start the RPM sequence on initialization
# NOTE: Disabled auto-start for development — manual control via ROS topic
# self.start_sequence()
```

> NOTE: Don't forget to colcon build any modification of the code and source the repo 