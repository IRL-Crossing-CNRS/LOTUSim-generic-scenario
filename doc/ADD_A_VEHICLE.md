# Add a vehicle

A new vehicle touches three repositories: the **core** (its Gazebo model and
its xdyn model), this **scenario workspace** (the agent class that spawns and
commands it) and the **Unity project** (what it looks like). The WAM-V is the
most recent vehicle added and the example used throughout; follow its files
side by side.

| Step | Repository | Files | Needed for |
|---|---|---|---|
| 1. Gazebo model | core | `assets/models/<name>/model.config`, `model.sdf`, `meshes/` | always |
| 2. xdyn model | core | `assets/models/<name>/<name>.yaml` | real hydrodynamics only |
| 3. Agent class | scenario | `src/lotusim_sdk/lotusim_sdk/agents/entity/physical/<name>.py`, `setup.py` | always |
| 4. xdyn server | scenario | `scenario_launch.sh`, `XDYN_CONFIGS` | real hydrodynamics only |
| 5. Unity prefab | Unity | a prefab, an Addressables entry | the 3D view |
| 6. Try it | scenario | a scenario JSON | always |

`<name>` is one lowercase word used everywhere: the model folder, the agent's
`MODEL_NAME`, and the Unity Addressables address. Here, `wamv`.

## 1. Gazebo model (core)

`assets/models/wamv/` in the core:

- `model.config`: name, SDF file, author.
- `model.sdf`: a `<model name="wamv">` with a `base_link`. Collision and
  visual meshes go in `meshes/` and are referenced as `model://wamv/meshes/...`.
- Sensors go inside a link: see the **Sensors** pages for each `gz:type`.
- Variants are other SDF files in the same folder (`model-battery.sdf`), picked
  by `"sdf_file"` in the scenario.

The model holds no physics plugin: the agent adds the physics interface block
when it spawns the vehicle (step 3).

## 2. xdyn model (core)

Only for a vehicle that should move under real hydrodynamics. Without it the
vehicle runs **Kinematic**: it follows its commanded velocity exactly.

`assets/models/wamv/wamv.yaml` describes the body for xdyn: mass and inertia,
added mass, the hull mesh for hydrostatics and wave forces, damping, and one
force model per actuator. Each actuator's name and command make the command
key the agent sends: `wamv_prop_left` with `commands: [T]` is commanded as
`wamv_prop_left(T)`.

The format, the pitfalls (the trailing `/` of `XDYN_ASSETS_PATH`, silently
ignored misindented keys) and links to xdyn's own documentation are on the
core's **xdyn** page (`docs/XDYN.md`). Start from the closest existing model.

## 3. Agent class (scenario workspace)

`src/lotusim_sdk/lotusim_sdk/agents/entity/physical/wamv.py`:

```python
from lotusim_sdk.agents.physical_entity import PhysicalEntity


class Wamv(PhysicalEntity):
    MODEL_NAME = "wamv"          # model folder in the core, and Unity address
    XDYN_PORT = 12348            # its xdyn server; None for Kinematic only
    DOMAINS = ["Surface"]        # "Surface", "Underwater" or "Aerial"
    THRUSTERS = []
    # Every command key of the xdyn model, at zero, so the first xdyn step
    # already carries them all.
    INITIAL_COMMANDS = {"wamv_prop_left(T)": 0.0, "wamv_prop_right(T)": 0.0}
```

Register it in `src/lotusim_sdk/setup.py`, under `lotusim.agents`:

```python
"wamv = lotusim_sdk.agents.entity.physical.wamv:Wamv",
```

The scenario's `"class"` is matched to that name once both are lowercased and
CamelCase is split with underscores: `Wamv` and `wamv` both find `wamv`,
`Bluerov2Heavy` and `Bluerov2_heavy` both find `bluerov2_heavy`. A vehicle class can also live in its own package (see
`src/external_packages/custom_task_demo/`): any installed wheel that declares
`lotusim.agents` is found. Rebuild the workspace after adding it
(`colcon build`), the registry is read from the installed packages.

To steer it, the GNC tasks need an allocation that turns a force and moment
into the vehicle's command keys: see **Navigation, guidance & control**
(`GNC_MODULAR_ARCHITECTURE.md`) and the existing allocations.

## 4. xdyn server (scenario workspace)

In `src/simulation_run/executable/scenario_launch.sh`, one line maps the class
to its model file and port:

```bash
XDYN_CONFIGS["Wamv"]="$LOTUSIM_PATH/assets/models/wamv/wamv.yaml 12348"
```

The port must be free and equal to the class's `XDYN_PORT`. The key is the
class name as scenarios write it. `xdyn_dt()` in the same script gives the
xdyn time step: 0.2 s by default.

## 5. Unity prefab (Unity project)

The renderer spawns a vehicle from the Addressables address equal to the
agent's `MODEL_NAME`.

1. Import the mesh and make a prefab of it.
2. Turn its pivot so the bow points along the direction of travel: the
   procedure is in the core's **Coordinate frames** page
   (`docs/coordinate_frames.md`, "Adding a new vehicle").
3. Add the prefab to the `SpawnInterface` Addressables group with the address
   `wamv` (Window > Asset Management > Addressables > Groups).
4. Rebuild the Addressables, then the player, and ship it: see the Unity
   project's **Build and ship the player** page.

Without this step the simulation still runs: the vehicle is simply not drawn.

## 6. Try it

A scenario JSON with one agent of the new class:

```json
{
  "world_file": "energy.world",
  "renderer_unity": true,
  "agents": [
    { "id": "wamv0", "class": "Wamv", "xdyn": true,
      "spawn": { "x": 0.0, "y": 0.0, "z": 0.0, "yaw": 0.0 } }
  ]
}
```

With no mission it drifts: check that it floats at the right draught and
stays upright before giving it a mission. `multi_vehicle_examples/wamv_drift_only.json`
is that test for the WAM-V; `wamv_xdyn.json` drives it under thrust and
`wamv_kinematic.json` runs it Kinematic.
