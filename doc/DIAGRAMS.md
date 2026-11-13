# LOTUSim-generic-scenario - Documentation

## 1. Project Overview

**LOTUSim Generic Scenario** is a multi-agent maritime simulator built on the following stack:

- **ROS 2** (Jazzy) for inter-process communication
- **Gazebo Sim** for physics simulation and the 3D world, with two worlds per run: the scenario world (e.g. `energy`) and `aerialWorld`
- **XDyn** for the hydrodynamic model (Surface/Underwater, `xdyn-for-cs` over WebSocket)
- **PX4 SITL** (optional) as the autopilot of `X500` drones
- **Unity** (optional) for high-fidelity 3D rendering via `ros_tcp_endpoint`

The project simulates naval and offshore-energy scenarios with surface vehicles (Wamv, Fremm, Commando, Pha, DtmbHull), underwater vehicles (Lrauv, Bluerov2Heavy, Mine), aerial vehicles (X500), and environment agents (Wind, Wake, OceanCurrent).

> This workspace is a **ROS 2 overlay** on top of the core LOTUSim (`$LOTUSIM_PATH`, the LOTUSim clone). The core provides the Gazebo systems, the SDF/3D assets, and the custom messages (`lotusim_msgs`, `lotusim_sensor_msgs`).

---

## 2. Package Tree (`src/`)

| Package               | Language | Role                                                                                    |
|-----------------------|----------|-----------------------------------------------------------------------------------------|
| `lotusim_sdk`         | Python   | **The SDK** - agent class hierarchy, behaviour-tree engine, built-in tasks (GNC, inspection, PX4 offboard) |
| `simulation_run`      | Python   | **Host orchestrator** - entry point, lifecycle management, agent spawning, dynamic spawn service |
| `lotusim_client`      | Python   | **Remote launcher** - `run_agent` runs an agent from another machine against the host simulation |
| `external_packages/*` | Python   | **Vehicle packages** - thin vehicle subclasses (`*_inspection`, `lrauv_propeller`, `custom_task_demo`) and vehicle-specific GNC (`*_gnc`) |
| `gz_ros2_bridge`      | C++      | **Gazebo -> ROS 2 bridge** - simulation statistics (`stats_gz_to_ros_bridge`)         |

### Detailed structure of `simulation_run/`

```txt
simulation_run/
├── config/                     # Scenario JSON, grouped by subject
│   ├── basic_examples/         # Behaviour-tree fundamentals
│   ├── current_examples/       # Ocean-current models
│   ├── wind_wake_examples/     # Wind, wake, PX4 aerial drones
│   ├── wave_examples/          # Sea states shared by hulls and renderer
│   ├── multi_vehicle_examples/ # One scenario per vehicle class
│   ├── example_rosbag/         # Recording a ROS 2 bag
│   ├── facet_demo/             # Full-fleet turbine inspection
│   ├── iros_demo/              # The demo scenario (59 agents)
│   └── waypoints/              # Patrol waypoint data files
├── executable/
│   ├── scenario_launch.sh      # Bash entry point (the script to run)
│   └── fetch_unity_player.sh   # Downloads the Unity player of players.json
└── simulation_run/
    ├── main.py                 # Python entry point (ros2 run simulation_run main)
    ├── simulation_runner.py    # Full lifecycle orchestration, launches the Gazebo worlds
    ├── ros_manager.py          # ROS 2 component init, executor loop
    ├── agents_manager.py       # AgentsManager: JSON -> agent classes -> instances -> spawn
    ├── utils.py                # CLI args, JSON/SDF parsing, class discovery, shard selection
    ├── configs.py              # Re-exports WaypointFollowerConfig from the SDK
    ├── csv_recorder.py         # CsvRecorder: one CSV per vessel ("record_csv")
    ├── mission_watcher.py      # MissionWatcher: logs when every mission is complete
    ├── prepare_weather.py      # Builds the sea state for xdyn and the renderer
    ├── wave_server_node.py     # Publishes the waves, rewrites them when the weather changes
    └── dynamic_spawn/          # DynamicSpawnService: spawn/despawn into a running simulation
```

The agent classes themselves live in `lotusim_sdk/lotusim_sdk/agents/` (see section 3).

---

## 3. Class Diagram - Agent Hierarchy (`lotusim_sdk.agents`)

This diagram shows the inheritance hierarchy of agents, from the SDK base classes to the vehicle packages.

```mermaid
classDiagram
    direction TB

    class Node {
        <<rclpy.Node>>
    }

    class ABC {
        <<ABC>>
    }

    class Agent {
        <<abstract>>
        world_name: str
        qos_profile: QoSProfile
        get_unique_model_num() int
        set_missions(specs, tick_rate_hz) None
        missions_ready() bool
        destroy_node() None
    }

    Node <|-- Agent
    ABC <|-- Agent

    class Entity {
        num: int
        agent_name: str
        sdf_string: str
        model_name: str
        renderer_type_name: str
        domains: list
        thrusters: list
        xdyn_port: int
        current_pose: Pose
        mas_action_client: ActionClient
        lotus_param() str
        confirm_spawn(assigned_name: str) None
        send_single_mas_cmd(value, server_timeout_sec: float) Future
        send_single_mas_cmd_geo(lat, lon, alt) Future
        send_single_mas_cmd_pose(pose) Future
        send_single_delete_cmd(server_timeout_sec: float) Future
        start_pause(duration: float) None
        resume_agent() None
    }

    Agent <|-- Entity

    class PhysicalEntity {
        MODEL_NAME: str
        XDYN_PORT: int
        THRUSTERS: list
        DOMAINS: list
        INITIAL_COMMANDS: dict
        __init__(sdf_string, world_name, xdyn_enabled)
    }

    class FixedEntity {
        <<abstract>>
        MODEL_NAME: str
        __init__(sdf_string, world_name)
    }

    Entity <|-- PhysicalEntity
    Entity <|-- FixedEntity

    class Environment {
        <<abstract>>
        __init__(world_name)
    }

    Agent <|-- Environment

    class Bluerov2Heavy
    class Lrauv
    class Mine
    class Wamv
    class Fremm
    class Commando
    class Pha
    class DtmbHull

    class X500 {
        PX4_MODEL_NAME: str = "x500_px4"
        px4_enabled: bool
        px4_control: str
        px4_instance: int
        confirm_spawn(assigned_name: str) None
        destroy_node() None
    }

    PhysicalEntity <|-- Bluerov2Heavy
    PhysicalEntity <|-- Lrauv
    PhysicalEntity <|-- Mine
    PhysicalEntity <|-- Wamv
    PhysicalEntity <|-- Fremm
    PhysicalEntity <|-- Commando
    PhysicalEntity <|-- Pha
    PhysicalEntity <|-- DtmbHull
    PhysicalEntity <|-- X500

    class Wind
    class Wake
    class OceanCurrent

    Environment <|-- Wind
    Environment <|-- Wake
    Environment <|-- OceanCurrent

    class Bluerov2HeavyInspection
    class WamvInspection
    class X500Inspection
    class CustomTaskDemoAgent
    class Bluerov2HeavyCurrent
    class Bluerov2HeavyCurrentInspection

    class LrauvPropeller {
        control_lrauv: Subscription
        propeller_phases: list
        period_sec: float
        current_phase_index: int
        nav_timer: Timer
        start_sequence() None
        nav_update() None
        send_current_phase_command() None
        send_propeller_command(rpm: float, pd: float) None
        control_lrauv_callback(msg: Bool) None
    }

    Bluerov2Heavy <|-- Bluerov2HeavyInspection
    Bluerov2Heavy <|-- CustomTaskDemoAgent
    Wamv <|-- WamvInspection
    X500 <|-- X500Inspection
    Lrauv <|-- LrauvPropeller
    PhysicalEntity <|-- Bluerov2HeavyCurrent
    Bluerov2HeavyCurrent <|-- Bluerov2HeavyCurrentInspection
```

`Agent`, `Entity`, `PhysicalEntity`, `FixedEntity`, `Environment` and the vehicle and environment classes are in `lotusim_sdk`. The classes at the bottom come from `external_packages/` (`bluerov2_heavy_inspection`, `wamv_inspection`, `x500_inspection`, `custom_task_demo`, `bluerov_gnc`, `lrauv_propeller`).

### Agent summary table by domain

| Domain          | Vehicle classes (`MODEL_NAME`, `XDYN_PORT`)                                                         | Physics                          |
|-----------------|------------------------------------------------------------------------------------------------------|----------------------------------|
| **Underwater**  | `Bluerov2Heavy` (`bluerov2_heavy`, 12347), `Lrauv` (`lrauv`, 12346), `Mine` (`mine`, 12350)         | XDyn, or Kinematic               |
| **Surface**     | `DtmbHull` (`dtmb_hull`, 12345), `Wamv` (`wamv`, 12348), `Fremm` (`fremm`, 12349), `Pha` (`pha`, 12351), `Commando` (`commando`, 12352) | XDyn, or Kinematic |
| **Aerial**      | `X500` (`x500`, no XDyn)                                                                             | `aerialWorld` (optionally flown by PX4 SITL) |
| **Environment** | `Wind`, `Wake`, `OceanCurrent`                                                                       | no model, they publish the environment |

### Common pattern for vehicle classes

Each concrete vehicle class declares **only class-level constants** (`MODEL_NAME`, `XDYN_PORT`, `THRUSTERS`, `DOMAINS`), with no `__init__` override:

1. `PhysicalEntity.__init__()` reads the constants and wires XDyn, or for `Aerial` the ROS 2 physics connection to `aerialWorld`
2. `Entity.__init__()` sets up the SDF string, pose tracking, and the MAS command client shared per world
3. `Agent.__init__()` starts the behaviour-tree mission timer
4. `lotus_param()` builds the `<lotus_param>` XML sent with the spawn command

`X500` is the only vehicle with its own `__init__`: it adds the PX4 SITL lifecycle (see `doc/PX4_DRONES.md`).

### Dynamic discovery mechanism

Agent classes are **discovered at runtime** via **entry points** (group `lotusim.agents`). Each package declares its classes in `setup.py` -> `entry_points`: `lotusim_sdk` for the base vehicles and environment agents, each `external_packages/*` package for its own. `simulation_run.utils.find_agent_class_globally()` and `lotusim_client`'s `run_agent` load a class by normalized name. Behaviour-tree tasks are discovered the same way (group `lotusim.tasks`).

---

## 4. Class Diagram - `simulation_run` Package

This diagram details the orchestration modules and their relationships.

```mermaid
classDiagram
    class main {
        signal_handler(signum: int, frame: FrameType) None
        main() None
    }

    class simulation_runner {
        process: subprocess.Popen
        executor: MultiThreadedExecutor
        cleanup_done: bool
        lotusim_pids: List~int~
        build_launch_command(world_file: str, debug: bool, gui: bool) List~str~
        start_simulation_process(commands: List~str~) List~subprocess.Popen~
        wait_for_world(world_name: str, timeout_sec: float) bool
        run_simulation(world_file, agents, max_simulation_time, debug_mode, ..., shard, shards) Any
        stop_simulation(executor: Optional~Object~) None
        reset_gazebo_state() None
    }

    class ros_manager {
        shutdown_flag: bool
        shutdown_flag_lock: Lock
        initialize_ros_components(executor, agents, world_name, world_file, config_dir, shard, shards) AgentsManager
        run_executor(executor: Any, max_simulation_time: Optional~float~) None
    }

    class AgentsManager {
        agents: Dict~str, Any~
        add_agents(agents, world_name, executor, world_file, config_dir) None
        delete_agents() None
        get_agent(name: str) Optional~Any~
        spawn_one_agent(...) Any
        despawn_agent(agent_name: str, executor: Any) bool
        _process_single_agent_type(agent_type, agent_info, world_name, executor, spawn_queue) None
        _create_agent_instance(agent_class, agent_sdf, world_name, xdyn_enabled, agent_info) Any
        _px4_kwargs(agent_class, agent_info) Dict
        _register_agent(agent_node, executor, spawn_queue, pose) None
        _spawn_all_agents(spawn_queue, executor) None
    }

    class DynamicSpawnService {
        <<rclpy.Node>>
        spawn and despawn agents at runtime
    }

    class CsvRecorder {
        <<rclpy.Node>>
        one CSV per vessel
    }

    class MissionWatcher {
        <<rclpy.Node>>
        logs when every mission is complete
    }

    class utils {
        get_cli_args() Namespace
        select_shard(agents, shard: int, shards: int) Any
        parse_simulation_config(config: Dict~str, Any~) Tuple~str, Any~
        load_config_from_json(config_path: str) Dict~str, Any~
        get_world_name(world_file_name: str) str
        generate_random_pose(agent_first_domain: str) List~float~
        inject_first_ais_pose(agents, base_dir: str) None
        xml_equivalent(xml1: str, xml2: str) bool
        json_name_to_class_name(json_name: str) str
        normalize_agent_name(name: str) str
        find_agent_class_globally(agent_name: str) type
    }

    class Agent {
        <<abstract>>
        lotusim_sdk.agents
    }

    main --> simulation_runner : calls run_simulation()
    main --> utils : get_cli_args(), load_config_from_json(), inject_first_ais_pose()
    simulation_runner --> ros_manager : initialize_ros_components(), run_executor()
    simulation_runner --> utils : get_world_name()
    simulation_runner --> CsvRecorder : if "record_csv"
    simulation_runner --> MissionWatcher : shard 0
    ros_manager --> AgentsManager : new, add_agents()
    ros_manager --> DynamicSpawnService : new
    DynamicSpawnService --> AgentsManager : spawn_one_agent(), despawn_agent()
    AgentsManager --> utils : json_name_to_class_name(), find_agent_class_globally()
    AgentsManager --> Agent : creates instances
```

### Orchestration flow explanation

The `simulation_run` package runs in **3 phases**: startup, runtime, and shutdown. A scenario can be split across several agent processes (`--shards N`): shard 0 starts the Gazebo worlds and the scenario-wide observers, the other shards only host their share of the agents and join the running world. Here is the precise call order between modules:

#### Phase 1 - Startup (from `main.py` to agent spawning)

1. **`main.main()`** is the Python entry point, launched by `ros2 run simulation_run main`.
   - Calls `utils.get_cli_args()` to parse `--config`, `--debug`, `--gui`, `--shards` and `--shard`
   - Registers a signal handler on `SIGINT` (Ctrl+C) and `SIGTERM` (`kill`), so that both shut down cleanly
   - Calls `utils.load_config_from_json()` to load the JSON file, and `utils.inject_first_ais_pose()`
   - Calls `utils.parse_simulation_config()` to extract `world_file` and `agents`
   - Hands off to `simulation_runner.run_simulation()`

2. **`simulation_runner.run_simulation()`** orchestrates the complete lifecycle:
   - Initializes `rclpy` (the ROS 2 Python client library) and creates a `MultiThreadedExecutor` (the ROS 2 executor that runs all agent nodes in parallel)
   - Shard 0 only: calls `reset_gazebo_state()` -> kills old `gz sim` processes and removes `/tmp/gz/sim`
   - Extracts the `world_name` from the `.world` file via `utils.get_world_name()` (parses the SDF XML to retrieve the `name` attribute of the `<world>` tag)
   - Shard 0: calls `build_launch_command()` -> builds the `mise run --cd $LOTUSIM_PATH sim *.world` commands (always 2: the hardcoded `aerialWorld.world` first, then the scenario's world). Without mise and a core clone it falls back to the nix-installed `lotusim run`. Then `start_simulation_process()` launches Gazebo detached, or in separate terminals with `scenario_launch.sh --terminals`
   - Other shards: `wait_for_world()` waits for shard 0's world instead
   - Adds a `CsvRecorder` node if the scenario has `"record_csv"`
   - Hands off to `ros_manager.initialize_ros_components()` for agent initialization
   - Shard 0: adds a `MissionWatcher` node, which logs when every agent's mission is complete

3. **`ros_manager.initialize_ros_components()`** initializes the ROS 2 graph:
   - Creates an `AgentsManager` and calls `AgentsManager.add_agents()` with this shard's agents
   - Creates the `DynamicSpawnService` node, which spawns and despawns agents in the running simulation

4. **`AgentsManager.add_agents()`** iterates over each agent entry of the JSON:
   - For each entry: calls `_process_single_agent_type()` which:
     - Converts the JSON name to a Python class name via `utils.json_name_to_class_name()` (e.g., `"Dtmb_hull"` -> `"DtmbHull"`)
     - Dynamically loads the class via `utils.find_agent_class_globally()` (lookup in the entry points, group `lotusim.agents`)
     - For each instance: calls `_create_agent_instance()` (with `_px4_kwargs()` for classes that accept PX4 parameters), sets the agent's missions, then `_register_agent()` adds the node to the executor and to the spawn queue
   - Once all agents are created: calls `_spawn_all_agents()` -> sends every **`MASCmd.CREATE_CMD`** in one `MASCmdArray` goal (`/{world}/mas_cmd_array`), or one `send_single_mas_cmd()` per agent when the host has no array server. Each result calls the agent's `confirm_spawn()` with its final entity name

#### Phase 2 - Runtime (main loop)

1. **`ros_manager.run_executor()`** enters the main loop:
   - Calls `executor.spin_once(timeout_sec=0.1)` in a loop
   - The executor runs all registered nodes: each agent ticks its behaviour-tree missions once `missions_ready()` (spawn confirmed and own pose received), and reads its pose and sensors through the subscriptions shared per world
   - The loop stops when `shutdown_flag = True`, `rclpy.ok() = False`, or `max_simulation_time` is reached

#### Phase 3 - Shutdown (cleanup)

1. When the user presses **Ctrl+C** (or sends `SIGTERM`):
   - `main.signal_handler()` sets `shutdown_flag = True`, so `run_executor()` returns
   - The `finally` block of `run_simulation()`:
     - Calls `agents_manager.delete_agents()` -> each agent's `destroy_node()` (an `X500` also stops its PX4 SITL process)
     - Closes the `CsvRecorder` and `MissionWatcher` nodes
     - Calls `stop_simulation()`, which terminates the Gazebo processes and the bridge, shuts down the executor, then calls `rclpy.shutdown()`

### Role of `utils.py` and `configs.py`

- **`utils`** is a cross-cutting utility module used by `main`, `simulation_runner`, and `AgentsManager`. It is not part of the sequential flow; it is called on demand for: loading JSON, parsing SDF XML, selecting a shard's agents, generating random poses, and dynamically discovering agent classes.
- **`configs.py`** only re-exports `WaypointFollowerConfig`, which now lives in the SDK (`lotusim_sdk/tasks/waypoint_follower.py`).

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

    Note over Shell: ROS 2: the active environment (nix devshell),<br/>else /opt/ros/${ROS_DISTRO}/setup.bash
    Shell->>Shell: source $LOTUSIM_PATH/install/setup.bash (core)
    Shell->>Shell: source $LOTUSIM_SCENARIO_WS/install/setup.bash

    Note over Shell: Clean up old processes
    Shell->>Shell: pkill gz sim, simulation_run main, run_agent,<br/>ros_tcp_endpoint, wave server...

    Note over Shell: Parse JSON config
    Shell->>Shell: jq -> world_file=energy.world, renderer_unity=true
    Shell->>Shell: jq -> agents: Wake, Bluerov2_heavy_inspection,<br/>Wamv_inspection, X500_inspection
    Shell->>Shell: copy the config into scenario_logs/<timestamp>/config

    Note over Shell: Launch ROS TCP Endpoint (Unity bridge)
    Shell->>TCP: ros2 run ros_tcp_endpoint default_server_endpoint<br/>--address 0.0.0.0 --tcp_ip 127.0.0.1

    Note over Shell: Launch XDyn (none here: every demo_facet agent is kinematic)
    loop For each agent type with xdyn=true
        Shell->>Shell: lookup XDYN_CONFIGS[agent_type] -> yml_file, port
        Shell->>XDyn: xdyn-for-cs agent.yml<br/>--address 127.0.0.1 --port PORT --dt DT
    end

    Note over Shell: Launch Unity (downloaded from the release on first run)
    Shell->>Unity: lotusim_unity_executables/lotusimenergy_linux/*.x86_64

    Note over Shell: Launch Python simulation (shard 0, --shards N adds shards 1..N-1)
    Shell->>Main: ros2 run simulation_run main --config facet_demo/demo_facet.json<br/>--shards 1 --shard 0

    Main->>Utils: get_cli_args() -> config path
    Main->>Utils: load_config_from_json(demo_facet.json)
    Main->>Utils: inject_first_ais_pose(agents) : no AIS for this scenario
    Main->>Utils: parse_simulation_config(config)
    Utils-->>Main: world_file, agents

    Main->>Main: signal.signal(SIGINT, signal_handler)
    Main->>SimRunner: run_simulation(world_file, agents)

    SimRunner->>SimRunner: rclpy.init()  MultiThreadedExecutor()
    SimRunner->>SimRunner: reset_gazebo_state() -> pkill gz sim, rm /tmp/gz/sim

    SimRunner->>SimRunner: build_launch_command(energy.world)
    Note over SimRunner: Always two worlds, the aerial one first:<br/>mise run --cd $LOTUSIM_PATH sim aerialWorld.world<br/>mise run --cd $LOTUSIM_PATH sim energy.world<br/>(lotusim run when mise is missing)

    SimRunner->>Gazebo: mise run --cd $LOTUSIM_PATH sim *.world

    SimRunner->>Utils: get_world_name(energy.world) -> parse SDF XML
    SimRunner->>RosMgr: initialize_ros_components(executor, agents, "energy")

    RosMgr->>AgentsMgr: AgentsManager()
    RosMgr->>AgentsMgr: add_agents(agents, "energy", executor)

    loop For each agent type in the JSON
        AgentsMgr->>Utils: json_name_to_class_name("Wamv_inspection")
        AgentsMgr->>Utils: find_agent_class_globally("Wamv_inspection")<br/>-> entry_points(group="lotusim.agents")
        loop For each instance (e.g. Wamv_inspection)
            AgentsMgr->>Agent: agent_class(sdf, "energy", xdyn_enabled)
            Note over Agent: PhysicalEntity -> Entity -> Agent __init__:<br/>class constants, SDF, mission timer
            Note over Agent: Joins the per-world shared ActionClient(mas_cmd),<br/>poses subscription and discovery timer
            AgentsMgr->>AgentsMgr: executor.add_node(agent_node)
            AgentsMgr->>AgentsMgr: spawn_queue.append(agent, pose)
        end
    end

    Note over AgentsMgr: Spawn all agents
    AgentsMgr->>Gazebo: send_batch_mas_cmd(spawn_queue)<br/>-> one MASCmdArray goal on /{world}/mas_cmd_array
    alt No mas_cmd_array server on the host
        loop For each agent in spawn_queue
            AgentsMgr->>Agent: send_single_mas_cmd(pose)
            Agent->>Agent: lotus_param() -> XML <lotus_param>
            Agent->>Gazebo: ActionClient.send_goal_async()<br/>-> /{world}/mas_cmd (CREATE_CMD)
        end
    end
    Gazebo-->>Agent: spawn result -> confirm_spawn(final entity name)

    RosMgr-->>SimRunner: return agents_manager

    SimRunner->>RosMgr: run_executor(executor)
    Note over RosMgr: Infinite loop: executor.spin_once(0.1s)<br/>until shutdown_flag or SIGINT

    Note over Agent: Runtime loop
    Agent->>Agent: _tick_missions()<br/>-> ticks the behaviour-tree missions once missions_ready()
    Agent->>Agent: one shared subscription per world<br/>← /{world}/poses (VesselPositionArray)
    Agent->>Agent: _shared_discovery_tick() -> _sensor_callback()<br/>← /{world}/{agent}/{sensor}/*

    User->>Shell: CtrlC (SIGINT)
    Shell->>Main: signal_handler() -> shutdown_flag = True, run_executor() returns
    SimRunner->>AgentsMgr: delete_agents() -> each agent's destroy_node()
    SimRunner->>SimRunner: stop_simulation(executor)
    SimRunner->>SimRunner: SIGTERM the world processes, pkill gz sim, ros_gz_bridge<br/>(GNOME terminals only with --close-terminals)
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
        XDYN["XDyn (xdyn-for-cs)<br/>WebSocket, ports 12345-12352<br/>1 instance per agent type, or per agent with xdyn_port"]
        GAZEBO["Gazebo Sim<br/>scenario world (energy.world)<br/>+ LOTUSim systems"]
        GAZEBO_AERIAL["Gazebo Sim<br/>aerialWorld.world<br/>always launched, first"]
        PX4["PX4 SITL<br/>(optional, one per X500 with px4: true)"]
    end

    subgraph "Sea state (scenarios with waves or weather)"
        WAVES["wave_server_node<br/>(waves to the renderer and to xdyn)"]
    end

    subgraph "ROS 2 Python - simulation_run (host)"
        MAIN["main.py, shard 0<br/>(+ shards 1..N-1 with --shards)"]
        SR["simulation_runner.py<br/>(lifecycle manager)"]
        RM["ros_manager.py<br/>(executor + init)"]
        AM["AgentsManager<br/>(factory/registry)"]
        DYN["DynamicSpawnService"]
    end

    subgraph "ROS 2 Python - lotusim_client (remote, optional)"
        RA["run_agent"]
    end

    subgraph "lotusim_sdk agent instances"
        AGENTS["PhysicalEntity / Environment<br/>(Wamv, Bluerov2Heavy, X500, Wind, Wake, ...)"]
    end

    subgraph "3D Rendering"
        UNITY["Unity player<br/>(lotusimenergy_linux/*.x86_64)"]
        TCP_EP["ros_tcp_endpoint<br/>(ROS 2 ↔ Unity TCP bridge, one player)"]
    end

    USER --> LAUNCH
    CONFIG --> LAUNCH

    LAUNCH -->|"pkill + setup env"| LAUNCH
    LAUNCH -->|"ros2 run"| TCP_EP
    LAUNCH -->|"python3 -m"| WAVES
    LAUNCH -->|"detached, or --terminals"| XDYN
    LAUNCH -->|"executable"| UNITY
    LAUNCH -->|"ros2 run simulation_run main"| MAIN

    MAIN --> SR
    SR -->|"reset + launch both worlds"| GAZEBO_AERIAL
    SR -->|"reset + launch both worlds"| GAZEBO
    SR --> RM
    RM --> AM
    RM --> DYN
    AM -->|"instantiates"| AGENTS
    RA -->|"instantiates (remote machine)"| AGENTS

    AGENTS <-->|"MAS commands (ROS 2 actions),<br/>poses and sensors (ROS 2 topics)"| GAZEBO
    GAZEBO <-->|"WebSocket"| XDYN
    GAZEBO -->|"AerialMAS forwards aerial spawns,<br/>pose mirrored back"| GAZEBO_AERIAL
    AGENTS -->|"starts, per X500"| PX4
    PX4 <-->|"gz-transport"| GAZEBO_AERIAL
    AGENTS -->|"wind (ROS 2 topic)"| GAZEBO_AERIAL
    WAVES --> XDYN

    GAZEBO <-->|"ROS 2 topics"| TCP_EP
    WAVES -->|"ROS 2 topics"| TCP_EP
    TCP_EP <-->|"TCP socket"| UNITY
```

`gz_ros2_bridge`'s `stats_gz_to_ros_bridge` is not started by any scenario launch path; it has its own launch file (`launch/bridge_nodes.launch.py`).

---

## 7. ROS 2 Nodes, Topics, and Actions Architecture

### Nodes

| Node                                     | Description                                                                                                      |
|------------------------------------------|------------------------------------------------------------------------------------------------------------------|
| `/wamvinspection0_3f9a1c2b`, `/wind`...  | One Python node for **each** instantiated agent (inherits from `rclpy.node.Node` via the `Agent` class). An entity's node is named after its class and number plus a random suffix; its entity name (the JSON `id` plus its index, e.g. `x500_px40`) is set afterwards. An environment agent's node is its class name. |
| `/{world}/mas_node`                      | `MultiAgentSystem` (or `AerialMAS`): spawns and deletes the entities of the world.                               |
| `/{world}/physics_plugin`                | `PhysicsInterfacePlugin`: links each entity to its physics engine (XDyn, Kinematic, or the aerial world).        |
| `/physics_aerial_linker`                 | `ROS2Interface`: receives the poses of the aerial world for the mirrored aerial entities.                       |
| `/{world}/render_interface`              | `RenderPlugin`: publishes what Unity needs to create and move the 3D models.                                     |
| `/{world}/{world}_sensor_system`         | Global node managing the sensors integrated in Gazebo.                                                           |
| `/{world}/{world}_power_manager`         | Power subsystem (batteries).                                                                                      |
| `/{world}/waypoint_follower`             | `WaypointFollowerPlugin`: trajectory following inside Gazebo for the agents that use it.                         |
| `/aerialWorld/wind_regions_plugin`       | Applies the wind and the wind regions in the aerial world.                                                        |
| `DynamicSpawnService`, `CsvRecorder`, `MissionWatcher` | Host-side nodes of `simulation_run` (section 4).                                                  |

### Topics

| Topic                                | Message Type                          | Used By                                                            |
|--------------------------------------|---------------------------------------|--------------------------------------------------------------------|
| `/{world}/poses`                     | `lotusim_msgs/VesselPositionArray`    | Pose tracking for all agents (one shared subscription per world)   |
| `/{world}/renderer_poses` & `renderer_cmd` | `lotusim_msgs/...`              | Rendering data exchanges needed for Unity                          |
| `/{world}/vessel_cmd_array`          | `lotusim_msgs/VesselCmdArray`         | Low-level thruster commands (rpm, pitch/diameter)                  |
| `/{world}/{agent}/control_lrauv`     | `std_msgs/Bool`                       | Starts or stops the `LrauvPropeller` sequence                      |
| `/{world}/{vessel}/ais_sensor/ais`   | `lotusim_sensor_msgs/AIS`             | Publication of AIS data (lat, lon, SOG, heading)                   |
| `/{world}/{vessel}/imu_sensor/IMU`   | `sensor_msgs/Imu`                     | Basic inertial data (dynamically discovered)                       |
| `/{world}/{vessel}/waypoint_reached` | `lotusim_msgs/WaypointFollowerStatus` | Waypoint crossing notification + path status                       |
| `/{world}/{vessel}/camera`           | `sensor_msgs/Image`                   | Raw video stream from onboard cameras                              |
| `/aerialWorld/wind`                  | `lotusim_msgs/Wind`                   | Wind vector, written by the `Wind` agent, applied in the aerial world |
| `/defenseScenario/sim_stats`         | `lotusim_msgs/SimStats`               | Simulation timing, from `stats_gz_to_ros_bridge` (topic name hardcoded there; not started by the launcher) |

### Actions and Services

| Interface Name                | Design Pattern | Type / Msg                        | Used For                                                                                                        |
|-------------------------------|----------------|-----------------------------------|-----------------------------------------------------------------------------------------------------------------|
| `/{world}/mas_cmd_array`      | **Action**     | `lotusim_msgs/action/MASCmdArray` | Spawn or delete a batch of agents in one request. The default spawn path.                                        |
| `/{world}/mas_cmd`            | **Action**     | `lotusim_msgs/action/MASCmd`      | Spawn ("CREATE_CMD") or delete ("DELETE_CMD") a single agent. Used when the host has no `mas_cmd_array` server, and by `AerialMAS` to forward to `/aerialWorld/mas_cmd`. |
| `/{world}/{vessel}/waypoints` | **Service**    | `lotusim_msgs/srv/SetWaypoints`   | Used by the vessel agent to upload its list of waypoints to the Gazebo waypoint follower plugin.                |
