# Architecture

How the repository is organised: the two machines involved, the package tree,
the agent class hierarchy, and the orchestration flow that gets a scenario
from a JSON file to spawned agents in Gazebo.

> Scope note: this document covers **repository / code organisation only**.
---

## 1. Host vs. remote machines

LOTUSim runs across up to two machines on the same LAN, sharing one
`ROS_DOMAIN_ID` and talking only over ROS 2 / DDS.

```mermaid
flowchart LR
    subgraph HOST["Host machine"]
        GZ["Gazebo + LOTUSim systems<br/>(entity_manager, physics_engine_interface, ...)"]
        UNITY["Unity renderer"]
        SR["simulation_run<br/>(scenario launcher)"]
    end

    subgraph REMOTE["Remote machine (optional)"]
        RC["lotusim_client<br/>(run_agent)"]
    end

    SR -->|"spawns host-side agents"| GZ
    RC -->|"MASCmd action: spawn/delete"| GZ
    RC -->|"topics: poses, sensors, cmds"| GZ
    GZ <-->|"ROS topics"| UNITY
```

- **Host**: runs Gazebo, the LOTUSim systems, the Unity renderer, and launches
  scenarios from `src/simulation_run/config/*.json` via `simulation_run`.
  Built with `lotusim clean_build`
- **Remote**: runs only `lotusim_client.run_agent` — no Gazebo. It spawns an
  agent into the host's simulation over ROS 2 and ticks that agent's BT
  mission locally. Needs only ROS 2, Python, and the `deployment/` bundle
  (§6).
- **Key principle**: the same agent class + same mission JSON runs
  identically whichever machine launches it — only the Python process's
  location differs.

---

## 2. Package tree (`src/`)

| Package | Role |
|---|---|
| `lotusim_sdk` | **The SDK.** Agent class hierarchy, the BT engine, built-in tasks. Ships as a wheel; used identically host- and remote-side. |
| `lotusim_client` | **Remote launcher.** `run_agent` CLI: instantiates an agent from a JSON config and ticks it against a running host simulation. Ships as a wheel. |
| `simulation_run` | **Host orchestrator.** Launches Gazebo/Unity, parses scenario JSON, spawns the full agent set, dynamic spawn/despawn service. ROS 2 `ament_python` package, host-only. |
| `external_packages/*` | **Concrete agent packages** — one per vehicle/demo (`bluerov2_heavy_inspection`, `wamv_inspection`, `x500_inspection`, `lrauv_propeller`, `custom_task_demo`). Each is a standalone ROS 2 package discovered via entry points (§3, §4). |
| `gz_ros2_bridge` | C++ package: two standalone bridge executables between `gz-transport` and ROS 2 (§7). |
| `deployment/` | The remote colcon workspace bundle: build script, `lotusim_msgs` source, wheels output (§6). |

### `lotusim_sdk/lotusim_sdk/`

```txt
lotusim_sdk/
├── agents/
│   ├── agent.py                 # Agent: rclpy.Node + BT mission engine (root abstract class)
│   ├── entity/
│   │   ├── __init__.py          # Entity: SDF model, pose tracking, MAS spawn/delete, sensor discovery
│   │   ├── physical/            # One file per concrete vehicle (bluerov2_heavy.py, wamv.py, x500.py, ...)
│   │   └── fixed/                # FixedEntity: static infrastructure, no physics engine
│   ├── physical_entity.py       # PhysicalEntity: XDyn / physics_engine_interface wiring
│   ├── fixed_entity.py
│   └── environment/
│       ├── wind.py               # Wind: Environment agent driving the ambient wind vector (§9)
│       └── wake/                 # Wake: Environment agent, Larsen power + Blended wind regions + LCOE (§9)
├── bt/                            # Behaviour Tree engine: Status, BehaviorNode, Sequence, Parallel,
│                                   # Blackboard, build_tree, load_task_registry
├── tasks/                         # Built-in TaskAgent leaves: fault_inspection, check_battery_state,
│                                   # waypoint_follower (+ fault_inspection_assets/ model + YOLO server)
├── spawn_utils.py                 # Shared host/remote helpers: pull a spawn pose out of a mission's
│                                   # waypoint_follower task when no explicit pose is given
└── trajectory_providers.py        # TrajectoryProvider ABC + PatrolFileProvider / WaypointListProvider
```

### `simulation_run/simulation_run/`

```txt
simulation_run/
├── main.py                # Entry point (`ros2 run simulation_run main --config ...`)
├── simulation_runner.py   # Full lifecycle: rclpy/executor init, launches Gazebo, hands off, cleans up
├── ros_manager.py         # Builds the AgentsManager, registers the DynamicSpawnService, runs the spin loop
├── agents_manager.py      # AgentsManager: JSON -> agent classes -> instances -> spawn queue -> MASCmd
├── utils.py                # CLI args, JSON/SDF parsing, class discovery, random pose generation
├── configs.py              # Re-exports WaypointFollowerConfig from the SDK
├── csv_recorder.py         # CsvRecorder: pure observer node, one CSV/vessel (pose+battery+mission
│                           #   progress); enabled via the scenario JSON's "record_csv" key
└── dynamic_spawn/          # Runtime spawn/despawn of agents into an already-running sim (§5)
```

### `external_packages/*`

Every package follows the same **thin subclass** shape — a base vehicle class
plus a `renderer_type_name`, with all behaviour coming from the mission JSON:

```txt
external_packages/
├── bluerov2_heavy_inspection/   # Bluerov2Heavy, thin
├── wamv_inspection/             # Wamv, thin
├── x500_inspection/             # X500, thin (also clears `domains` to skip aerialWorld physics)
├── lrauv_propeller/              # Lrauv + standalone propeller RPM cycling logic (dev example, no BT)
└── custom_task_demo/             # Bluerov2Heavy + a custom TaskAgent wired in code via
                                   # self._missions.add_task(...) instead of scenario JSON
```

`lrauv_propeller` and `custom_task_demo` are intentionally **not** thin: they
show the two ways to extend an agent beyond "pure JSON mission" — raw
ROS pub/sub logic in the class itself, or a code-registered BT task.

---

## 3. Agent class hierarchy

```mermaid
classDiagram
    direction TB

    class Node { <<rclpy.Node>> }
    class ABC { <<ABC>> }

    class Agent {
        <<abstract>>
        world_name: str
        qos_profile: QoSProfile
        _missions: MissionSet
        _blackboard: Blackboard
        set_missions(specs, tick_rate_hz) None
        missions_ready() bool
        destroy_node() None
    }
    Node <|-- Agent
    ABC <|-- Agent

    class Environment {
        <<abstract>>
        __init__(world_name)
    }
    Agent <|-- Environment

    class Entity {
        num: int
        agent_name: str
        sdf_string: str
        model_name: str
        renderer_type_name: str
        current_pose: Pose
        mas_action_client: ActionClient
        lotus_param() str
        send_single_mas_cmd(value) Future
        send_single_delete_cmd() Future
        confirm_spawn(assigned_name) None
        missions_ready() bool
    }
    Agent <|-- Entity

    class PhysicalEntity {
        MODEL_NAME: str
        XDYN_PORT: int
        THRUSTERS: list
        DOMAINS: list
        _lotus_blocks() str
    }
    Entity <|-- PhysicalEntity

    class FixedEntity {
        <<abstract>>
        MODEL_NAME: str
    }
    Entity <|-- FixedEntity

    class Bluerov2Heavy
    class Wamv
    class Fremm
    class Commando
    class Pha
    class DtmbHull
    class Lrauv
    class Mine
    class X500

    PhysicalEntity <|-- Bluerov2Heavy
    PhysicalEntity <|-- Wamv
    PhysicalEntity <|-- Fremm
    PhysicalEntity <|-- Commando
    PhysicalEntity <|-- Pha
    PhysicalEntity <|-- DtmbHull
    PhysicalEntity <|-- Lrauv
    PhysicalEntity <|-- Mine
    PhysicalEntity <|-- X500

    class Wind {
        ambient wind vector (ENU)
    }
    class Wake {
        wake model: Larsen
        LCOE (optional)
        wind_regions (optional, Blended)
    }
    Environment <|-- Wind
    Environment <|-- Wake
```

### Concrete vehicle classes and their constants

| Class | `MODEL_NAME` | `XDYN_PORT` | `DOMAINS` |
|---|---|---|---|
| `Bluerov2Heavy` | `bluerov2_heavy` | 12347 | Underwater |
| `Lrauv` | `lrauv` | 12346 | Underwater |
| `Mine` | `mine` | 12350 | Underwater |
| `Fremm` | `fremm` | 12349 | Surface |
| `Commando` | `commando` | 12352 | Surface |
| `Wamv` | `wamv` | 12348 | Surface |
| `Pha` | `pha` | 12351 | Surface |
| `DtmbHull` | `dtmb_hull` | 12345 | Surface |
| `X500` | `x500` | `None` (native ROS 2, no XDyn) | Aerial |

Each concrete class declares **only these class-level constants** — no
`__init__` override. `PhysicalEntity.__init__` reads them and wires up XDyn
(or, for `Aerial`, the native ROS 2 physics connection); `Entity.__init__`
sets up the SDF string, pose tracking, and the shared MASCmd/discovery
infrastructure (§3.1); `Agent.__init__` starts the BT mission timer.

### 3.1 Per-process shared infrastructure (`agents/entity/__init__.py`)

To scale to large agent counts in one process, `Entity` does **not** give
each instance its own ActionClient, pose subscription, or discovery timer.
Instead, module-level registries share one of each per `world_name` across
every agent in the process:

- one `ActionClient` for `/{world}/mas_cmd`, so rclpy never cross-routes a
  goal/result meant for one agent to another,
- one subscription to `/{world}/poses`, parsed once per message into a
  `name -> pose` dict that each entity reads by `O(1)` lookup,
- one discovery timer per world that queries the ROS graph once a second and
  dispatches matching `/{world}/{agent}/...` topics to the owning entity.

This eliminates an `O(N²)` scaling bottleneck that previously starved the
process (including spawn confirmations) once agent counts got large.

`agent_name` is a **property**, not a plain attribute: its setter moves this
entity's entry in the shared registry above whenever the name changes (host
deconfliction via `confirm_spawn`, or a launcher assigning an `id`-based
name). Every rename therefore goes through one place — the discovery timer
and the pose/battery topic lookups always find the entity under its current
name, never the one it was constructed with.

### 3.2 Spawn confirmation

`Entity.missions_ready()` gates the agent's first mission tick on
`_spawn_confirmed and current_pose is not None`: the host is the single
authority on entity names (it deconflicts duplicates across machines), so an
agent only starts ticking its mission once it has adopted the host-assigned
name (`confirm_spawn`) and has seen its own pose on `/{world}/poses` under
that name.

---

## 4. Task and agent discovery (entry points)

Both agent classes and BT task classes are discovered **across every
installed wheel** via setuptools entry-point groups — there is no central
registry to edit when adding a new package.

| Entry-point group | Declared in | Consumed by |
|---|---|---|
| `lotusim.agents` | each package's `setup.py` (e.g. `lotusim_sdk`, `bluerov2_heavy_inspection`, `custom_task_demo`) | `simulation_run.utils.find_agent_class_globally()`, `lotusim_client.run_agent._find_agent_class()` |
| `lotusim.tasks` | `lotusim_sdk` (built-ins: `fault_inspection`, `check_battery_state`, `waypoint_follower`) + any package defining its own `TaskAgent` | `lotusim_sdk.bt.builder.load_task_registry()` |

A new vehicle or task added in its own wheel is found automatically by both
the host launcher and the remote `run_agent` — neither needs to import it.

---

## 5. Host orchestration flow (`simulation_run`)

```mermaid
sequenceDiagram
    actor User
    participant Shell as scenario_launch.sh
    participant Main as main.py
    participant Utils as utils.py
    participant SimRunner as simulation_runner.py
    participant Gazebo as Gazebo
    participant RosMgr as ros_manager.py
    participant AgentsMgr as AgentsManager
    participant DynSpawn as DynamicSpawnService
    participant Agent as Agent instances

    User->>Shell: ./scenario_launch.sh --config my_scenario.json
    Shell->>Main: ros2 run simulation_run main --config ...

    Main->>Utils: load_config_from_json() / inject_first_ais_pose() / parse_simulation_config()
    Utils-->>Main: world_file, agents, aerial_enabled
    Main->>SimRunner: run_simulation(world_file, agents, ...)

    SimRunner->>SimRunner: rclpy.init(); MultiThreadedExecutor()
    SimRunner->>SimRunner: reset_gazebo_state()
    SimRunner->>Gazebo: terminal -> lotusim run *.world
    SimRunner->>RosMgr: initialize_ros_components(executor, agents, world_name, ...)

    RosMgr->>AgentsMgr: AgentsManager(); add_agents(agents, ...)
    RosMgr->>DynSpawn: create + executor.add_node() (subscribes /spawn_cmd, /despawn_cmd)

    loop for each agent type/instance
        AgentsMgr->>Utils: json_name_to_class_name() / find_agent_class_globally()
        AgentsMgr->>Agent: instantiate; set_missions() if JSON has "missions"
        AgentsMgr->>AgentsMgr: executor.add_node(agent); queue for spawn
    end
    AgentsMgr->>Agent: send_single_mas_cmd(pose) for every queued agent
    Agent->>Gazebo: MASCmd action (CREATE_CMD) with lotus_param() SDF blocks

    RosMgr-->>SimRunner: return agents_manager
    SimRunner->>RosMgr: run_executor(executor)
    Note over RosMgr: executor.spin_once(0.1s) loop, ticking every Agent's BT mission timer

    User->>Shell: Ctrl+C
    SimRunner->>AgentsMgr: delete_agents() (DELETE_CMD + destroy_node() per agent)
    SimRunner->>SimRunner: kill Gazebo/terminal/bridge processes, executor.shutdown(), rclpy.shutdown()
```

The **remote** side (`lotusim_client.run_agent`) runs the equivalent of the
inner loop only: it resolves agent classes the same way, calls
`set_missions()`, spawns via the same `send_single_mas_cmd`/MASCmd action
against the host's Gazebo, spins its own `MultiThreadedExecutor`, and on
SIGINT/SIGTERM halts BT roots and sends `send_single_delete_cmd()` before
shutting down.

---

## 6. Global architecture (all processes)

```mermaid
flowchart TB
    subgraph "User Input"
        USER["User"]
        CONFIG["Scenario JSON"]
        LAUNCH["scenario_launch.sh"]
    end

    subgraph "Physics (external processes)"
        XDYN["XDyn (WebSocket, ports 12345-12352)<br/>one instance per agent type"]
        GAZEBO["Gazebo Sim<br/>SDF world + LOTUSim systems"]
    end

    subgraph "ROS2 Python - simulation_run (host)"
        MAIN["main.py"]
        SR["simulation_runner.py"]
        RM["ros_manager.py"]
        AM["AgentsManager"]
        DYN["DynamicSpawnService"]
    end

    subgraph "ROS2 Python - lotusim_client (remote, optional)"
        RA["run_agent"]
    end

    subgraph "lotusim_sdk agent instances"
        AGENTS["PhysicalEntity / FixedEntity / Environment<br/>(Lrauv, Wamv, Bluerov2Heavy, Wind, ...)"]
    end

    subgraph "ROS2 - gz_ros2_bridge"
        STATS["stats_gz_to_ros_bridge"]
        WIND_B["wind_ros_to_gz_bridge"]
    end

    subgraph "3D Rendering"
        UNITY["Unity renderer"]
        TCP_EP["ros_tcp_endpoint"]
    end

    USER --> LAUNCH
    CONFIG --> LAUNCH
    LAUNCH -->|"terminal"| XDYN
    LAUNCH -->|"terminal"| GAZEBO
    LAUNCH -->|"executable"| UNITY
    LAUNCH -->|"ros2 run"| TCP_EP
    LAUNCH -->|"ros2 run simulation_run main"| MAIN

    MAIN --> SR --> GAZEBO
    SR --> RM --> AM
    RM --> DYN
    AM -->|"instantiates"| AGENTS
    RA -->|"instantiates (remote machine)"| AGENTS

    AGENTS <-->|"MASCmd action, poses/sensor topics"| GAZEBO
    GAZEBO <-->|"WebSocket"| XDYN
    GAZEBO <-->|"gz::transport"| STATS
    GAZEBO <-->|"gz::transport"| WIND_B

    AGENTS <-->|"ROS2 topics"| TCP_EP
    TCP_EP <-->|"TCP socket"| UNITY
```

---

## 7. `gz_ros2_bridge` (C++)

Two standalone executables (no shared library), each linking
`gz-transport`/`gz-msgs` and `rclcpp`/`lotusim_msgs`:

| Node | Direction | Purpose |
|---|---|---|
| `stats_gz_to_ros_bridge` | Gazebo → ROS 2 | Publishes sim time / Real-Time Factor as `lotusim_msgs/SimStats`. |
| `wind_ros_to_gz_bridge` | ROS 2 → Gazebo | Injects wind commands from ROS 2 into `gz::transport`. |

`launch/bridge_nodes.launch.py` kills any previous instances of both, then
launches them together after a short delay. It is a convenience entry point
only — no scenario launch path uses it. `wind_ros_to_gz_bridge` is started
unconditionally by `simulation_runner.start_wind_bridge()` and torn down in
`stop_simulation()`. It is the sole link between the ROS-side wind vector
(`/aerialWorld/wind`) and `gz-sim-wind-effects-system` — a translator, idle
until something publishes that topic (§9).

---

## 8. Dynamic spawn subsystem (`simulation_run/dynamic_spawn/`)

Lets you spawn or remove agents from an **already-running** simulation,
independent of the scenario that was initially launched.

- `service.py` — `DynamicSpawnService` node: subscribes `/spawn_cmd` (JSON
  `{"AgentType": {...}}`) and `/despawn_cmd` (agent name), publishes status
  on `/spawn_status`, and exposes a `/list_agents` (`std_srvs/Trigger`)
  service. Delegates to `AgentsManager.spawn_one_agent()` /
  `despawn_agent()`.
- `spawn_agent.py`, `despawn_agent.py`, `list_agents.py` — thin CLI wrappers
  (`ros2 run simulation_run spawn_agent ...`) around the same topics/service.

---

## 9. Wind and wake (`agents/environment/`)

Two `Environment` agents (no SDF model, no MAS spawn), split along the one line
that matters: **who writes the wind, and who reads it**.

`/aerialWorld/wind` (`lotusim_msgs/Wind`) is the system's ambient/global wind
topic. Three parties touch it, and none of them know about each other:

| | Role |
|---|---|
| Unity wind sliders | write it, by hand |
| `Wind` agent (`wind.py`) | writes it, from the scenario; also subscribes to it (see below) |
| `wind_regions` Gazebo plugin (core repo, `systems/wind_regions/`) | reads it → forces on wind-enabled links |
| `Wake` agent (`wake/`) | reads it → turbine power, LCOE (ambient only, never regions) |

On top of that, `/aerialWorld/wind/regions` (`lotusim_msgs/WindRegionArray`)
carries an optional list of 2D wind regions, all altitudes, each with its own
vector, that override the ambient wind inside their footprint. `WindRegion`
carries one of two shapes via a `shape_type` discriminator plus an embedded
sub-message (ROS `.msg` has no inheritance/union, so this is the usual
tagged-union pattern): `WindRegionBox` — an axis-aligned box `(x1,y1)`–
`(x2,y2)` — or `WindRegionConeSegment` — a tapered frustum (`origin`, unit
downstream `axis`, `length`, `r_start`, `r_end`). **Two agents can write this
topic**, and neither knows about the other: `wind.py`'s own static `regions`
list (always the box shape), and `wake.py`'s dynamic `wind_regions` block
(the wake footprint itself, as chained cone segments — see below). It is
latched with no merging between publishers, so declaring both in one scenario
means whichever publishes last silently blanks out the other's regions.
`wind_regions` (the Gazebo plugin) reads the topic without caring which agent
wrote what is on it, or which shape any given region uses.

- `wind.py` — the `Wind` node holds one ENU vector and, **only while a mission
  is actively running**, republishes it on `/aerialWorld/wind` at
  `publish_rate_hz`. `set_wind`/`wait` BT tasks in its `missions` change it. A
  mission root defaults to `RUNNING` before its first tick, so the topic is
  claimed the instant `missions` is set, not after; once every root reaches
  `SUCCESS`/`FAILURE` the agent goes silent again and the sliders regain the
  topic immediately. With no `missions` at all it never publishes the ambient
  vector — a `Wind` agent declared without one is a legitimate no-op, there
  purely so wind missions can be added later without introducing a new agent.
  It also holds an optional static `regions` list (constructor kwarg), always
  republished on `/aerialWorld/wind/regions` regardless of mission state. A
  region can set `mirror_global: true` instead of a static vector, in which
  case its published vector is always `-1 *` the ambient vector currently in
  effect — to know that value even while passive (sliders driving
  `/aerialWorld/wind` instead of the agent), `wind.py` also subscribes to its
  own `/aerialWorld/wind` topic. That subscription intentionally uses plain
  (volatile) QoS rather than the publisher's TRANSIENT_LOCAL: the Unity
  sliders' publisher durability isn't controlled by this repo, and a
  TRANSIENT_LOCAL subscriber against a VOLATILE publisher would silently
  receive nothing (DDS QoS incompatibility), not just miss the replay.
- `wake/wake_model_base.py` — `WakeModelBase`, holding turbine and inflow
  parameters (diameter, thrust/power coefficients, cut-in/cut-out speed,
  ambient turbulence intensity, shear exponent) with abstract `power()` /
  `wind_speeds_full()`. Positions are ENU throughout: a turbine is `(x, y, z)`
  with `z` the hub height, a wind vector is `[vx, vy]`.
- `wake/larsen.py` — `LarsenWakeModel`, the one concrete model. A
  semi-analytical Larsen (2009) wake ported from the IRL Crossing benchmark
  repository [`lotusim-wake-models`](https://github.com/IRL-Crossing-CNRS/lotusim-wake-models)
  (`models/larsen.py` @ 8c18232), where it was validated against OpenFOAM v8 +
  turbinesFoam actuator-line CFD on the NREL 5MW. It replaced the Jensen and
  Gaussian models this package used to carry, beating both on every power
  protocol. It combines wakes by sequential *local* superposition — each
  upstream deficit applies to the already-reduced speed, in downstream order —
  rather than by the root-sum-square of freestream deficits the old models
  used, which is what stops deep rows saturating. Two departures from upstream
  are marked in the file: the ENU frame (upstream's vertical axis is `y`), and
  yaw derating by the *absolute* projection onto the turbines' facing axis, so
  that a southerly wind produces power instead of zeroing the farm — LOTUSim's
  wind direction is slider-driven and cannot be assumed northward. Locked
  against the published benchmark by `test/test_larsen_wake.py`.
- `wake/blended.py` — `BlendedWakeModel`, a Larsen/Gaussian blend used only for
  the *spatial* wake field (`wind_regions` below), never for power. Also
  ported from `lotusim-wake-models` (`models/blended.py` @ 8c18232), and also
  wraps a `LarsenWakeModel` instance — the same one `Wake` already built for
  power — rather than building its own, so rotor geometry stays one source of
  truth. One departure from upstream: `farm_velocity_at_point` projects onto
  the *current* wind vector before walking turbines, where upstream's spatial
  queries assume wind blows along one fixed axis (true for the single-direction
  CFD runs they were calibrated against, not for a slider-driven vector).
- `wake/wake_regions.py` — `WakeRegionGenerator` and `wind_changed_enough`,
  pure Python (no rclpy). Represents each turbine's wake footprint as a
  handful of `WindRegionConeSegment`s **chained end to end** — one segment's
  `r_end` is the next one's `r_start`, so the geometry is one continuous
  tapered cone, closed-form point-in-cone tested directly on the Gazebo side,
  rather than a stack of boxes circumscribing the real (circular) wake
  cross-section. That shape depends only on rotor geometry (diameter, ct,
  ambient_ti), not on wind speed — every calibrated deficit term in
  `BlendedWakeModel` is linear in freestream speed, so the threshold crossing
  used to find each segment's radius doesn't depend on it either — so it is
  computed once, at construction, and only rotated/translated for the current
  wind direction on each update. Segment *count* is now purely a
  velocity-gradient granularity knob (each segment already tapers smoothly,
  and chains without a seam into the next), unlike the box stack it replaced
  where segment count also had to double as a visual-smoothness knob. The
  downstream chain itself is capped by `max_downstream_diameters`, not by
  that same threshold going to zero: the calibrated centreline fit decays so
  slowly (`0.58 * (x/D)^-0.35`) that a 5% deficit is not reached until
  roughly `x/D=1100`, so the cap is what actually determines wake length for
  any realistic setting — see `Wake`'s `wind_regions` docs in
  WRITE_SCENARIO.md. `wind_changed_enough` gates how often the whole thing is
  regenerated: doing it on every wind message would mean reshuffling every
  wake segment on a topic the Gazebo plugin re-scans per wind-enabled link,
  per physics tick, for a wind vector that may have barely moved.
- `wake/wake.py` — the `Wake` node: subscribes `/aerialWorld/wind` (ambient
  only), publishes per-turbine effective speed and power on
  `/{world}/wind/turbines`, and — if its config block has an `lcoe` section —
  integrates that power into produced energy, discovers any spawned agent's
  `*/battery/state` topic for the robot operating cost, and publishes an LCOE
  (AUD/MWh) estimate on `/{world}/lcoe`. LCOE lives inside `Wake` because it is
  a direct integral of the farm power computed there. With a `wind_regions`
  section (requires `wake_model: "larsen"`) it also becomes the second
  possible writer of `/aerialWorld/wind/regions` — see above — publishing the
  `WakeRegionGenerator` output for the current wind, subject to
  `wind_changed_enough`'s hysteresis.

**No bridge process.** The `wind_regions` Gazebo System plugin (core repo)
embeds its own `rclcpp::Node` and subscribes to both ROS topics above
directly — it replaces stock `gz-sim-wind-effects-system` (which only supports
one uniform global wind and can't express regions) and the
`wind_ros_to_gz_bridge` translator that used to forward `/aerialWorld/wind`
into it over gz-transport. For each link tagged wind-enabled
(`<enable_wind>true</enable_wind>`, same convention stock WindEffects used),
the plugin resolves a wind vector by testing the link's world X/Y against the
region list — shape-agnostic, box or cone segment, via each `RegionState`'s
own `Contains()` — last matching region wins, otherwise the ambient vector —
and applies a force `scaling_factor * (wind - link_velocity)` via
`Link::AddWorldForce`.

---

## 10. The deployment bundle (`deployment/`)

`deployment/` **is** the remote colcon workspace:

```txt
deployment/
├── build_wheels.sh      # Builds dist/lotusim_sdk-*.whl and dist/lotusim_client-*.whl (host-side)
├── dist/                 # Wheel output
├── src/
│   ├── lotusim_msgs/     # Shipped as SOURCE — compiled rosidl typesupport is tied to the exact
│   │                     #   ROS distro + CPython version, so it's rebuilt on the remote
│   └── example_agent/    # Reference template package for a new remote agent
├── my_config.json        # Example remote scenario config
└── README.md
```

Full instructions: [`deployment/README.md`](../deployment/README.md); the
wheel/source split rationale is in the root
