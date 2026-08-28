# LOTUSim-generic-scenario - Documentation

## 1. Project Overview

**LOTUSim Generic Scenario** is a multi-agent maritime simulator built on the following stack:

- **ROS 2** (Humble/Jazzy) for inter-process communication
- **Gazebo** for physics simulation and the 3D world
- **XDyn** for the hydrodynamic model (Surface/Underwater via WebSocket)
- **Unity** (optional) for high-fidelity 3D rendering via `ros_tcp_endpoint`

The project enables simulation of naval scenarios with surface vehicles (Fremm, Commando, Wamv, Pha, DtmbHull), underwater vehicles (Lrauv, BlueROV2Heavy, Mine), and aerial vehicles (X500).

> This workspace is a **ROS 2 overlay** on top of the core LOTUSim installed in `~/lotusim_ws`. The core provides physics, SDF/3D assets, and custom messages (`lotusim_msgs`, `lotusim_sensor_msgs`).

---

## 2. Package Tree (`src/`)

| Package               | Language | Role                                                                                    |
|-----------------------|----------|-----------------------------------------------------------------------------------------|
| `simulation_run`      | Python   | **Orchestration core** - entry point, lifecycle management, agent spawning              |
| `agents/*`            | Python   | **Concrete agents** - each vehicle type is an independent ROS 2 package                 |
| `gz_ros2_bridge`      | C        | **Gazebo ↔ ROS 2 bridges** - simulation statistics and wind vector                     |

### Detailed structure of `simulation_run/`

```txt
simulation_run/
├── config/                     # Scenario JSON, grouped by subject
│   ├── basic_examples/         # Behaviour-tree fundamentals
│   ├── current_examples/       # Ocean-current models
│   ├── wind_wake_examples/     # Wind, wake, PX4 aerial drones
│   ├── multi_vehicle_examples/ # One scenario per vehicle class
│   ├── facet_demo/             # Full-fleet turbine inspection
│   └── waypoints/              # Patrol waypoint data files
├── executable/
│   └── scenario_launch.sh      # Bash entry point (the script to run)
└── simulation_run/
    ├── main.py                 # Python entry point (ros2 run simulation_run main)
    ├── simulation_runner.py    # Full lifecycle orchestration
    ├── ros_manager.py          # ROS2 component init, executor loop
    ├── agents_manager.py       # Agent management
    ├── agent.py                # Abstract Agent class (base for all agents)
    ├── utils.py                # Utilities (JSON, SDF, entry points, lotus_param XML)
    └── configs.py              # WaypointFollowerConfig dataclass
```

---

## 3. Class Diagram - `agents/` Package

This diagram shows the complete inheritance hierarchy of agents, with their key attributes and methods.

```mermaid
classDiagram
    direction TB

    class Node {
        <<Node>>
    }

    class ABC {
        <<ABC>>
    }

    class Agent {
        <<abstract>>
        agent_name: str
        world_name: str
        sdf_string: str
        current_pose: Pose
        last_pose_update: float
        sensor_buffers: dict
        sensors_subscribers: list
        qos_profile: QoSProfile
        discovery_timer: Timer
        _subscribed_topics: set
        mas_action_client: ActionClient
        pose_subscription: Subscription

        __init__(sdf_string: str, world_name: str, xdyn_port: int) Agent
        get_first_domain() str
        _poses_callback(msg: VesselPositionArray) None
        send_single_mas_cmd(value, server_timeout_sec: float) Future
        send_single_mas_cmd_geo(lat: float, lon: float, alt: float) Future
        send_single_mas_cmd_pose(pose: Pose) Future
        send_single_delete_cmd(server_timeout_sec: float) Future
        lotus_param()* str
        _discover_and_subscribe_topics() None
        _sensor_callback(msg: ROS2 msg, buffer_name: str, topic_name: str) None
        start_pause(duration: float) None
        resume_agent() None
    }

    Node <|-- Agent
    ABC <|-- Agent

    class Lrauv {
        _next_model_num: int
        model_name: str = "lrauv"
        domains: list = ["Underwater"]
        thrusters: list = ["propeller"]
        xdyn_port: int = 12346
        get_unique_model_num() int
        lotus_param() str
    }

    class Bluerov2Heavy {
        _next_model_num: int
        model_name: str = "bluerov2_heavy"
        domains: list = ["Underwater"]
        thrusters: list = ["thruster1", "thruster2", ..., "thruster8"]
        xdyn_port: int = 12347
        get_unique_model_num() int
        lotus_param() str
    }
    
    class Mine {
        _next_model_num: int
        model_name: str = "mine"
        domains: list = ["Underwater"]
        thrusters: list = [""]
        xdyn_port: int = 12350
        get_unique_model_num() int
        lotus_param() str
    }

    class Fremm {
        _next_model_num: int
        model_name: str = "fremm"
        domains: list = ["Surface"]
        thrusters: list = [""]
        xdyn_port: int = 12349
        get_unique_model_num() int
        lotus_param() str
    }

    class Commando {
        _next_model_num: int
        model_name: str = "commando"
        domains: list = ["Surface"]
        thrusters: list = [""]
        xdyn_port: int = 12352
        get_unique_model_num() int
        lotus_param() str
    }

    class Wamv {
        _next_model_num: int
        model_name: str = "wamv"
        domains: list = ["Surface"]
        thrusters: list = [""]
        xdyn_port: int = 12348
        get_unique_model_num() int
        lotus_param() str
    }

    class Pha {
        _next_model_num: int
        model_name: str = "pha"
        domains: list = ["Surface"]
        thrusters: list = [""]
        xdyn_port: int = 12351
        get_unique_model_num() int
        lotus_param() str
    }

    class DtmbHull {
        _next_model_num: int
        model_name: str = "dtmb_hull"
        domains: list = ["Surface"]
        thrusters: list = [""]
        xdyn_port: int = 12345
        get_unique_model_num() int
        lotus_param() str
    }

    class X500 {
        _next_model_num: int
        model_name: str = "x500"
        domains: list = ["Aerial"]
        thrusters: list = [""]
        xdyn_port: int = None
        get_unique_model_num() int
        lotus_param() str
    }

    Agent <|-- Lrauv
    Agent <|-- Bluerov2Heavy
    Agent <|-- Fremm
    Agent <|-- Commando
    Agent <|-- Wamv
    Agent <|-- Pha
    Agent <|-- DtmbHull
    Agent <|-- Mine
    Agent <|-- X500

    class LrauvPropeller {
        vesselCmd_pub: Publisher
        control_lrauv: Subscription
        propeller_phases: list
        period_sec: float
        current_phase_index: int
        nav_timer: Timer
        start_sequence() None
        nav_update() None
        send_current_phase_command() None
        send_propeller_command(rpm: float, pd: float) None
        control_lrauv_callback(msg: bool) None
    }

    Lrauv <|-- LrauvPropeller
```

### Agent summary table by domain

| Domain         | Base Agents                                    | Physics                   |
|----------------|------------------------------------------------|---------------------------|
| **Underwater** | `Lrauv`, `Bluerov2_heavy`, `Mine`              | XDynWebSocket             |
| **Surface**    | `Wamv`, `Fremm`, `Commando`, `Pha`, `DtmbHull` | XDynWebSocket             |
| **Aerial**     | `X500`                                         | Native ROS 2 (no XDyn)    |

### Common pattern for base agents

Each concrete agent follows the exact same pattern:

1. `_next_model_num` (class variable) for auto-incrementing the instance number
2. `get_unique_model_num()` (classmethod) returns and increments the counter
3. `init()` - sets `num`, `model_name`, `renderer_type_name`, `xdyn_port/ip`, `domains[]`, then calls `super().__init__()`
4. `lotus_param()` - delegates to `utils.generate_lotus_param()` with agent-specific parameters

### Dynamic discovery mechanism

Agent classes are **dynamically discovered at runtime** via **ROS 2 entry points** (group `lotusim.agents`). Each agent package declares its classes in `setup.py` -> `entry_points`. The function `utils.find_agent_class_globally()` loads the class by normalized name.

---

## 4. Class Diagram - `simulation_run` Package

This diagram details the orchestration modules and their relationships.

```mermaid
classDiagram
    class main {
        executor: MultiThreadedExecutor
        shutdown_flag: bool
        shutdown_flag_lock: Lock
        signal_handler(signum: int, frame: FrameType) None
        main() None
    }

    class simulation_runner {
        process: subprocess.Popen
        cleanup_done: bool
        lotusim_pids: List~int~
        build_launch_command(world_file: str, debug: bool) List~str~
        start_simulation_process(commands: List~str~) List~subprocess.Popen~
        run_simulation(world_file: str, agents: Dict~str, Any~, max_simulation_time: Optional~float~, debug_mode: bool) Any
        stop_simulation(executor: Optional~Object~) None
        reset_gazebo_state() None
    }

    class ros_manager {
        shutdown_flag: bool
        shutdown_flag_lock: Lock
        initialize_ros_components(executor: Any, agents: Dict~str, Any~, world_name: str) AgentsManager
        run_executor(executor: Any, max_simulation_time: Optional~float~) None
    }

    class AgentsManager {
        agents: Dict~str, Any~
        add_agents(agents: Dict~str, Any~, world_name: str, executor: Any) None
        delete_agents() None
        get_agent(name: str) Optional~Any~
        _process_single_agent_type(agent_type: str, agent_info: Dict~str, Any~, world_name: str, executor: Any, spawn_queue: List~Tuple(Any, Any)~) None
        _create_agent_instance(agent_class: Any, agent_sdf: str, world_name: str, xdyn_enabled: bool, agent_info: Dict~str, Any~) Any
        _register_agent(agent_node: Any, executor: Any, spawn_queue: List~Tuple(Any, Any)~, pose: Any) None
        _spawn_all_agents(spawn_queue: List~Tuple(Any, Any)~) None
    }

    class utils {
        get_cli_args() Namespace
        parse_simulation_config(config: Dict~str, Any~) Tuple~str, Dict[str, Any], bool~
        load_config_from_json(config_path: str) Dict~str, Any~
        get_world_name(world_file_name: str) str
        generate_random_pose(agent_first_domain: str) List~float~
        inject_first_ais_pose(agents: dict) None
        generate_lotus_param(renderer_type_name: str, domains: List~str~, thrusters: List~str~, xdyn_ip: str?, xdyn_port: str?, trajectory_follower: str?, trajectory_follower_config: WaypointFollowerConfig?) str
        xml_equivalent(xml1: str, xml2: str) bool
        json_name_to_class_name(json_name: str) str
        normalize_agent_name(name: str) str
        find_agent_class_globally(agent_name: str) type
    }

    class WaypointFollowerConfig {
        <<dataclass>>
        guidance_mode: str?
        loop: bool?
        range_tolerance: float?
        linear_accel_limit: float?
        angular_accel_limit: float?
        linear_velocities_limits: Tuple~float, float~?
        angular_velocities_limits: float?
        linear_pid: Tuple~float, float, float~?
        angular_pid: Tuple~float, float, float~?
    }

    class Agent {
        <<abstract>>
        ...
    }

    main --> simulation_runner : calls run_simulation()
    main --> utils : get_cli_args(), load_config(), inject_first_ais_pose()
    simulation_runner --> ros_manager : initialize_ros_components()
    simulation_runner --> utils : get_world_name()
    ros_manager --> AgentsManager : new add_agents()
    AgentsManager --> utils : json_name_to_class_name(), find_agent_class_globally()
    AgentsManager --> Agent : creates instances
    utils --> WaypointFollowerConfig : used in generate_lotus_param()
```

### Orchestration flow explanation

The `simulation_run` package runs in **3 phases**: startup, runtime, and shutdown. Here is the precise call order between modules:

#### Phase 1 - Startup (from `main.py` to agent spawning)

1. **`main.main()`** is the Python entry point, launched by `ros2 run simulation_run main`.
   - Calls `utils.get_cli_args()` to parse `--config` and `--debug`
   - Calls `utils.load_config_from_json()` to load the JSON file
   - Calls `utils.parse_simulation_config()` to extract `world_file`, `agents`, and `aerial_enabled`
   - Registers a signal handler on `SIGINT` (the Unix signal sent by Ctrl+C) to intercept shutdown and perform a clean cleanup
   - Hands off to `simulation_runner.run_simulation()`

2. **`simulation_runner.run_simulation()`** orchestrates the complete lifecycle:
   - Initializes `rclpy` (the ROS 2 Python client library, which allows creating nodes, publishers, subscribers, etc.) and creates a `MultiThreadedExecutor` (the ROS 2 executor that runs all agent nodes in parallel)
   - Calls `reset_gazebo_state()` -> kills old `gz sim` processes and removes `/tmp/gz/sim`
   - Calls `build_launch_command()` -> builds the `lotusim run *.world` commands (always 2: the hardcoded aerial world, then the scenario's world)
   - Calls `start_simulation_process()` -> launches Gazebo in separate terminals
   - Extracts the `world_name` from the `.world` file via `utils.get_world_name()` (parses the SDF XML to retrieve the `name` attribute of the `<world>` tag)
   - Hands off to `ros_manager.initialize_ros_components()` for agent initialization

3. **`ros_manager.initialize_ros_components()`** initializes the ROS 2 graph:
   - Creates an `AgentsManager`
   - Calls `AgentsManager.add_agents()` with the complete agent dictionary

4. **`AgentsManager.add_agents()`** iterates over each agent type from the JSON:
   - For each type: calls `_process_single_agent_type()` which:
     - Converts the JSON name to a Python class name via `utils.json_name_to_class_name()` (e.g., `"Dtmb_hull"` -> `"DtmbHull"`)
     - Dynamically loads the class via `utils.find_agent_class_globally()` (lookup in ROS 2 entry points, group `lotusim.agents`)
     - For each instance (`nb_agents` times):
       - Calls `_create_agent_instance()` -> instantiates the class with the SDF, world, and xdyn flag. If a `trajectory` is present in the JSON, it is injected via the constructor.
       - Calls `_register_agent()` -> registers the node in the ROS 2 executor and adds it to the spawn queue
   - Once all agents are created: calls `_spawn_all_agents()` -> for each agent, calls `agent.send_single_mas_cmd(pose)` which sends a **ROS 2 action `MASCmd.CREATE_CMD`** to Gazebo to spawn the vehicle in the world

#### Phase 2 - Runtime (main loop)

1. **`ros_manager.run_executor()`** enters the main loop:
   - Calls `executor.spin_once(timeout_sec=0.1)` in an infinite loop
   - The executor runs all registered `Agent` nodes, triggering their timers and callbacks:
     - `_discover_and_subscribe_topics()` - dynamically discovers sensor topics
     - `_poses_callback()` - receives poses from Gazebo
     - `_sensor_callback()` - buffers sensor messages
   - The loop stops when `shutdown_flag = True`, `rclpy.ok() = False`, or `max_simulation_time` is reached

#### Phase 3 - Shutdown (cleanup)

1. When the user presses **Ctrl+C**:
   - `main.signal_handler()` sets `shutdown_flag = True` and calls `simulation_runner.stop_simulation()`
   - `stop_simulation()`:
     - Calls `agents_manager.delete_agents()` -> sends `MASCmd.DELETE_CMD` for each agent
     - Kills Gazebo, terminal, and bridge processes
     - Removes all nodes from the executor, shuts down the executor, then calls `rclpy.shutdown()`

### Role of `utils.py` and `configs.py`

- **`utils`** is a cross-cutting utility module used by `main`, `simulation_runner`, and `AgentsManager`. It is not part of the sequential flow; it is called on-demand for: loading JSON, parsing SDF XML, resolving SDF file paths, generating `<lotus_param>` XML, and dynamically discovering agent classes.
- **`WaypointFollowerConfig`** (in `configs.py`) is a simple `dataclass` that structures the waypoint follower plugin parameters. It is used exclusively by `utils.generate_lotus_param()` when an agent needs a trajectory follower.

---

## 5. Sequence Diagram - Launching the `facet_demo/demo_facet.json` scenario

This diagram shows **everything that happens** when the user enters:

```bash
./src/simulation_run/executable/scenario_launch.sh --config facet_demo/demo_facet.json
```

```mermaid
sequenceDiagram
    actor User
    participant Shell as scenario_launch.sh
    participant XDyn as XDyn Physics
    participant Unity as Unity Renderer
    participant TCP as ros_tcp_endpoint
    participant Main as main.py
    participant Utils as utils.py
    participant SimRunner as simulation_runner.py
    participant Gazebo as Gazebo
    participant RosMgr as ros_manager.py
    participant AgentsMgr as AgentsManager
    participant Agent as Agent

    User->>Shell: ./scenario_launch.sh --config facet_demo/demo_facet.json

    Note over Shell: OS detection -> ROS Distro (humble/jazzy)
    Shell->>Shell: source /opt/ros/${DISTRO}/setup.bash
    Shell->>Shell: source ~/lotusim_ws/install/setup.bash
    Shell->>Shell: source LOTUSim-generic-scenario/install/setup.bash

    Note over Shell: Clean up old processes
    Shell->>Shell: pkill xdyn, gzserver, gzclient, ros2...

    Note over Shell: Parse JSON config
    Shell->>Shell: jq -> renderer_unity=true
    Shell->>Shell: jq -> agents: Lrauv, Bluerov2_heavy, Mine, Pha,...

    Note over Shell: Launch ROS TCP Endpoint (Unity bridge)
    Shell->>TCP: terminal -> ros2 run ros_tcp_endpoint<br/>--address 0.0.0.0 --tcp_ip 127.0.0.1

    Note over Shell: Launch XDyn (1 process per agent type with xdyn=true)
    loop For each agent type with xdyn=true
        Shell->>Shell: lookup XDYN_CONFIGS[agent_type] -> yml_file, port
        Shell->>XDyn: terminal -> xdyn-for-cs agent.yml<br/>--address 127.0.0.1 --port PORT --dt 0.2
    end

    Note over Shell: Launch Unity
    Shell->>Unity: ./lotusim_scenario.x86_64

    Note over Shell: Launch Python simulation
    Shell->>Main: ros2 run simulation_run main --config facet_demo/demo_facet.json

    Main->>Utils: get_cli_args() -> config path
    Main->>Utils: load_config_from_json(demo_facet.json)
    Main->>Utils: inject_first_ais_pose(agents) : no AIS for this scenario
    Main->>Utils: parse_simulation_config(config)
    Utils-->>Main: world_file, agents, aerial_enabled=true

    Main->>Main: signal.signal(SIGINT, signal_handler)
    Main->>SimRunner: run_simulation(world_file, agents, aerial=true)

    SimRunner->>SimRunner: rclpy.init()  MultiThreadedExecutor()
    SimRunner->>SimRunner: reset_gazebo_state() -> pkill gz sim, rm /tmp/gz/sim

    SimRunner->>SimRunner: build_launch_command(defenseScenario.world, aerial=true)
    Note over SimRunner: Commands: lotusim run aerialWorld.world<br/> lotusim run defenseScenario.world

    SimRunner->>Gazebo: terminal -> lotusim run *.world

    SimRunner->>Utils: get_world_name(defenseScenario.world) -> parse SDF XML
    SimRunner->>RosMgr: initialize_ros_components(executor, agents, "defenseScenario", aerial=true)

    RosMgr->>AgentsMgr: AgentsManager()
    RosMgr->>AgentsMgr: add_agents(agents, "defenseScenario", executor, aerial=true)

    loop For each agent type in the JSON
        AgentsMgr->>Utils: json_name_to_class_name("Lrauv") -> "Lrauv"
        AgentsMgr->>Utils: find_agent_class_globally("Lrauv")<br/>-> entry_points(group="lotusim.agents")
        loop For nb_agents instances (e.g. Lrauv)
            AgentsMgr->>Agent: agent_class(sdf, "defenseScenario", xdyn_enabled)
            Note over Agent: __init__: set num, model_name, domains, xdyn_port<br/>-> super().__init__() -> Node.__init__()
            Note over Agent: Creates: ActionClient(mas_cmd)<br/>Subscription(poses), Timer(discovery)
            AgentsMgr->>AgentsMgr: executor.add_node(agent_node)
            AgentsMgr->>AgentsMgr: spawn_queue.append(agent, pose)
        end
    end

    Note over AgentsMgr: Spawn all agents
    loop For each agent in spawn_queue
        AgentsMgr->>Agent: send_single_mas_cmd(pose)
        Agent->>Agent: lotus_param() -> XML <lotus_param>
        Agent->>Gazebo: ActionClient.send_goal_async()<br/>-> /{world}/mas_cmd (CREATE_CMD)
    end

    RosMgr-->>SimRunner: return agents_manager

    SimRunner->>RosMgr: run_executor(executor)
    Note over RosMgr: Infinite loop: executor.spin_once(0.1s)<br/>until shutdown_flag or SIGINT

    Note over Agent: Runtime loop
    Agent->>Agent: _discover_and_subscribe_topics()<br/>-> dynamically subscribes to sensor topics
    Agent->>Agent: _poses_callback()<br/>← /{world}/poses (VesselPositionArray)
    Agent->>Agent: _sensor_callback()<br/>← /{world}/{agent}/{sensor}/*

    User->>Shell: CtrlC (SIGINT)
    Shell->>Main: signal_handler()
    Main->>SimRunner: stop_simulation(executor)
    SimRunner->>AgentsMgr: delete_agents() -> DELETE_CMD
    SimRunner->>SimRunner: pkill terminal, gz sim, ros_gz_bridge
    SimRunner->>SimRunner: executor.shutdown(), rclpy.shutdown()
```

---

## 6. Global Architecture Diagram

Full system view showing all processes, their interconnections, and the protocols used.

```mermaid
flowchart TB
    subgraph "User Input"
        USER["User"]
        CONFIG["JSON config<br/>(demo_facet.json)"]
        LAUNCH["scenario_launch.sh"]
    end

    subgraph "Physics (external processes)"
        XDYN["XDyn<br/>(WebSocket, ports 12345-12352)<br/>1 instance per agent type"]
        GAZEBO["Gazebo Sim<br/>(gz sim)<br/>SDF world + LOTUSim plugins"]
        GAZEBO_AERIAL["Gazebo Aerial<br/>(aerialWorld.world)<br/>always launched"]
    end

    subgraph "ROS2 Python - simulation_run"
        MAIN["main.py<br/>(entry point)"]
        SR["simulation_runner.py<br/>(lifecycle manager)"]
        RM["ros_manager.py<br/>(executor + init)"]
        AM["AgentsManager<br/>(factory/registry)"]
        AGENTS["Agent instances<br/>(Lrauv, Fremm, BlueROV2, Mine, etc.)"]
    end

    subgraph "ROS2 - gz_ros2_bridge"
        STATS["StatsGzToRos2Bridge<br/>(stats -> ROS2)"]
        WIND["WindRos2ToGzBridge<br/>(ROS2 -> wind)"]
    end

    subgraph "3D Rendering"
        UNITY["Unity<br/>(lotusim_scenario.x86_64)"]
        TCP_EP["ros_tcp_endpoint<br/>(ROS ↔ Unity TCP bridge)"]
    end

    USER --> LAUNCH
    CONFIG --> LAUNCH

    LAUNCH -->|"pkill + setup env"| LAUNCH
    LAUNCH -->|"terminal"| XDYN
    LAUNCH -->|"terminal"| GAZEBO
    LAUNCH -->|"terminal"| GAZEBO_AERIAL
    LAUNCH -->|"executable"| UNITY
    LAUNCH -->|"ros2 run"| TCP_EP
    LAUNCH -->|"ros2 run simulation_run main"| MAIN

    MAIN --> SR
    SR -->|"reset + launch Gazebo"| GAZEBO
    SR --> RM
    RM --> AM
    AM -->|"instantiates"| AGENTS

    AGENTS <-->|"MAS commands<br/>(ROS2 action)"| GAZEBO
    AGENTS <-->|"poses, sensors<br/>(ROS2 topics)"| GAZEBO
    GAZEBO <-->|"WebSocket"| XDYN

    GAZEBO <-->|"gz::transport"| STATS
    GAZEBO_AERIAL <-->|"gz::transport"| WIND

    AGENTS <-->|"ROS2 topics"| TCP_EP
    TCP_EP <-->|"TCP socket"| UNITY
```

---

## 7. ROS 2 Nodes, Topics, and Actions Architecture

### Nodes

| Node                                     | Description                                                                                                      |
|------------------------------------------|------------------------------------------------------------------------------------------------------------------|
| `/lrauv0`, `/commando0`...               | An independent Python node for **each** instantiated agent (inherits from `rclpy.node.Node` via the `Agent` class). |
| `/{world}/gz_entity_management_node`     | Internal Gazebo node managing the lifecycle (entity spawning/deletion) of models in the world.                   |
| `/{world}/physics_plugin`                | Manages the orchestration of Gazebo's base physics simulation (time step, etc.).                                 |
| `/{world}/render_interface`              | Specific bridge communicating with the rendering interface (used to maintain the link with Unity via TCP).       |
| `/{world}/defenseScenario_sensor_system` | Global node managing the sensors integrated in Gazebo.                                                           |
| `/{world}/waypoint_follower`             | Node attached to the Gazebo plugin managing trajectory following for agents (`CommandoTrajectoryFollower`).      |

### Topics

| Topic                                | Message Type                          | Used By                                                            |
|--------------------------------------|---------------------------------------|--------------------------------------------------------------------|
| `/{world}/poses`                     | `lotusim_msgs/VesselPositionArray`    | Pose tracking for all agents (read in `_poses_callback`)           |
| `/{world}/renderer_poses` & `cmd`    | `lotusim_msgs/...`                    | Rendering data exchanges needed for Unity                          |
| `/{world}/lotusim_vessel_array_cmd`  | `lotusim_msgs/VesselCmdArray`         | Low-level thruster commands (rpm, pitch/diameter)                  |
| `/{world}/{agent}/control_lrauv`     | `std_msgs/Bool`                       | Starts or stops the propulsion sequence                            |
| `/{world}/{vessel}/ais_sensor/ais`   | `lotusim_sensor_msgs/AIS`             | Publication of AIS data (lat, lon, SOG, heading)                   |
| `/{world}/{vessel}/imu_sensor/IMU`   | `sensor_msgs/Imu`                     | Basic inertial data (dynamically discovered)                       |
| `/{world}/{vessel}/waypoint_reached` | `lotusim_msgs/WaypointFollowerStatus` | Waypoint crossing notification + path status                       |
| `/{world}/{vessel}/camera`           | `sensor_msgs/Image`                   | Raw video stream from onboard cameras                              |
| `/defenseScenario/sim_stats`         | `lotusim_msgs/SimStats`               | Monitoring simulation timing performance            |
| `/aerialWorld/wind`                  | `lotusim_msgs/Wind`                   | Command to dynamically inject wind into the aerial world           |

### Actions and Services

| Interface Name                | Design Pattern | Type / Msg                        | Used For                                                                                                        |
|-------------------------------|----------------|-----------------------------------|-----------------------------------------------------------------------------------------------------------------|
| `/{world}/mas_cmd`            | **Action**     | `lotusim_msgs/action/MASCmd`      | Spawn ("CREATE_CMD") or delete ("DELETE_CMD") a single agent and its 3D model in the simulator.                 |
| `/{world}/mas_cmd_array`      | **Action**     | `lotusim_msgs/action/MASCmdArray` | Handle a batch of spawns/deletions in a single request to optimize network performance.                         |
| `/{world}/{vessel}/waypoints` | **Service**    | `lotusim_msgs/srv/SetWaypoints`   | Used by the vessel agent to upload its list of waypoints to the Gazebo application plugin.     |
| `.../change_state`            | **Service**    | `lifecycle_msgs/srv/ChangeState`  | Manages the official ROS lifecycle (unconfigured -> active) of each agent's sensors (AIS, IMU).                 |