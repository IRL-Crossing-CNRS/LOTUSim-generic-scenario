# Developer guide

Entry point for someone who is going to **change** this repository, rather
than run scenarios in it. If you only want to run a scenario, the top-level
[README.md](../README.md) is enough and this file is not.

The reference documents are already written and are not repeated here; this
guide says where each piece lives, which seam to use for a given kind of
change, and which conventions will bite you if you ignore them.

## 1. Reading order

| Read | When |
|---|---|
| [ARCHITECTURE.md](ARCHITECTURE.md) | Always first. Package tree, agent class hierarchy, entry-point discovery, host orchestration flow, host vs. remote |
| [MISSIONS.md](MISSIONS.md) | Before writing a behaviour-tree task. Engine, task lifecycle, tick semantics |
| [WRITE_SCENARIO.md](WRITE_SCENARIO.md) | Full scenario JSON reference — every key of every built-in task |
| [GNC_MODULAR_ARCHITECTURE.md](GNC_MODULAR_ARCHITECTURE.md) | Before touching Navigation / Guidance / Control / Allocation, or plugging your own algorithm into one of them |
| [DIAGRAMS.md](DIAGRAMS.md) | Class, sequence and node/topic diagrams, when a written description is not enough |
| [ACCELERATED_SIMULATION.md](ACCELERATED_SIMULATION.md) | Running faster than real time |
| [WAKE_EFFECT.md](WAKE_EFFECT.md) | Turbine wake and wind regions |
| [../deployment/README.md](../deployment/README.md) | Running an agent from a machine without Gazebo |

The C++ side — Gazebo plugins, the xdyn bridge, the message definitions, the
vehicle models and worlds — is the **core** repository, `LOTUSim`
(`~/lotusim_ws/src/LOTUSim`). Its own `docs/DEVELOPER.md` maps
that side. Anything below about topics or message schemas is defined there,
in `interfaces/lotusim_msgs/`.

## 2. Where the code is

```txt
src/
├── lotusim_sdk/          the SDK: agent classes, BT engine, generic tasks, control library
├── simulation_run/       host orchestrator: reads the scenario JSON, launches everything
├── lotusim_client/       remote launcher (run one agent against a running host sim)
├── external_packages/    one package per vehicle/demo, plus the four *_gnc packages
└── gz_ros2_bridge/       C++ gz-transport <-> ROS 2 bridge executable
```

Inside the SDK, the parts a GNC or mission developer touches:

```txt
lotusim_sdk/lotusim_sdk/
├── agents/               Agent -> Entity -> PhysicalEntity -> one file per vehicle
├── bt/                   behaviour-tree engine (status, nodes, composites, blackboard, builder)
├── tasks/                the vehicle-agnostic task leaves, including the GNC blocks
└── control/              algorithm library used by the tasks, not ROS-aware:
                          guidance.py (LOS, pure pursuit), pid.py, frames.py (NED/ENU),
                          current_feedforward.py (model-based current feedforward)
```

The split between `tasks/` and `control/` is deliberate: `control/` holds
plain Python with no ROS dependency and is unit-testable on its own;
`tasks/` wraps it in the ROS/BT plumbing. Put new algorithms in `control/`
and keep the task a thin adapter.

## 3. The four extension points

### Add a behaviour-tree task

Subclass `TaskAgent` (`lotusim_sdk/tasks/base.py`), implement `on_enter` /
`update` / `on_exit`, and declare it in your package's `setup.py` under the
`lotusim.tasks` entry-point group. Nothing central needs editing: the
registry is built by scanning installed wheels
(`lotusim_sdk.bt.builder.load_task_registry()`). The scenario JSON then
refers to it by the registry name. Full lifecycle contract in
[MISSIONS.md](MISSIONS.md) §4.

### Add a vehicle

Add a class under `agents/entity/physical/`, subclassing `PhysicalEntity`,
and declare it under `lotusim.agents`. It needs an SDF model in the core
repo's `assets/models/`. If it is to run on real hydrodynamics, it also needs
an xdyn YAML there and an entry in `scenario_launch.sh`'s `XDYN_CONFIGS` map
(model file + a port unique to that vehicle class). A vehicle with no xdyn
model runs Kinematic (`"xdyn": false`).

### Plug your own GNC algorithm

Replace one node, keep the topics. Each block is defined by exactly one
subscribe/publish pair, listed in
[GNC_MODULAR_ARCHITECTURE.md](GNC_MODULAR_ARCHITECTURE.md) §2. A new guidance
law that publishes `lotusim_msgs/GuidanceSetpoint` on
`/<world>/<agent>/guidance` is drop-in: Control and Allocation cannot tell
the difference, and switching to it is a one-line `task` change in the
scenario JSON.

Write the block as a new task rather than a new branch inside an existing
one. Which law runs is a scenario-level choice, not a runtime mode.

### Add a current model

Two different places, and picking the wrong one is the classic mistake:

- **The disturbance the vehicle actually feels** is injected on the *host*
  side, in the core repo: either inside xdyn (its own `ekman current`
  environment model, selected by which BlueROV YAML is loaded), or in
  `xdyn_websocket.cpp` by the Galilean velocity-shift trick
  (`gauss_markov_current.cpp`, `copernicus_current.cpp`). Kinematic vehicles
  get a separate, much simpler uniform drift through
  `ocean_current_feed.cpp`.
- **The model the controller believes in** is here, in
  `lotusim_sdk/control/current_feedforward.py`, selected by the `feedforward`
  params block of the `control` task.

They are independent on purpose: holding the injected current fixed while
changing only the controller's model is what makes a model-vs-model
comparison attributable to the model.

## 4. Conventions

- **Frames.** xdyn and every GNC task work in **NED**: `z` positive
  downwards, heading positive clockwise seen from above. Gazebo, Unity and
  the `/poses` bus are **ENU**. Conversions live in
  `lotusim_sdk/control/frames.py` — use them; do not open-code a sign flip.
  For an aerial vehicle this means an altitude of 40 m above the surface is
  `desired_depth = -40`.
- **Registry names are API.** A task's entry-point name appears in every
  scenario JSON that uses it. Renaming one breaks scenarios silently at
  build time, not at launch. `bluerov_gnc`'s `bluerov_navigation` and
  `bluerov_guidance_*` names survive as thin aliases of the generic tasks for
  exactly this reason.
- **Rebuild, then re-source.** `colcon build && source install/setup.bash`
  after any Python change: the scenario launcher runs the *installed* wheel,
  not your working tree. The core repo is rebuilt separately with
  `lotusim clean_build`, never with a bare `colcon build`.
- **Empty `external_packages/*/` directories** left behind by a branch switch
  are invisible to `git status` (git does not track empty directories) but
  visible to `colcon`. Delete them if a build complains about a package with
  no `setup.py`.

## 5. Verifying a change

A scenario that starts and does not crash proves almost nothing: several
past bugs — an xdyn `external forces:` key silently parsed as empty, a
diverging allocator gain — produced clean startup and a wrong trajectory.

Add `"record_csv": true` at the scenario's top level and read
`scenario_logs/<timestamp>/csv/<agent>.csv` afterwards. Check that position
and attitude go where the mission says, over a run long enough for a
divergence to show (60–90 s, not 10 s). `scenario_logs/<timestamp>/config/`
holds a snapshot of the exact scenario, world and agent files that ran, so a
result stays reproducible after the live config changes.

Always test through `scenario_launch.sh` or the `lotusim` CLI. A bare
`gz sim` run bypasses the orchestration this repository is about, so it
cannot confirm that a change works.

## 6. Known gaps

Listed so nobody re-derives them from the code:

- No live path source: `SetWaypoints` exists as a message type, but no
  Guidance task serves it. Missions are static scenario JSON.
- No aerial rotor-mixer Allocation on this branch: X500 in the generic
  pipeline runs through `kinematic_allocation`; the real mixer is work in
  progress elsewhere. PX4 SITL is the separate, non-GNC path.
- Navigation is ground truth: no sensor model, no estimation error.
- WAMV, dtmb_hull and the hull-only surface vessels have no xdyn thruster
  model, so they are drift-only under xdyn by nature.

The per-vehicle detail is in
[../src/simulation_run/config/README.md](../src/simulation_run/config/README.md).
