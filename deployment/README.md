# LOTUSim — Remote Agent Deployment

This folder contains everything needed to run a LOTUSim agent from a machine
that does **not** have the full LOTUSim simulation stack installed.

The agent node runs on the remote machine and communicates with Gazebo over
the ROS 2 network (both machines must be on the same LAN with the same `ROS_DOMAIN_ID`).

---

## Prerequisites (remote machine)

- Ubuntu 24.04
- ROS 2 Jazzy installed and sourced (`source /opt/ros/jazzy/setup.bash`)
- Python 3.12+
- `pip` and `colcon` available (colcon ships with ROS 2)
- ROS geographic messages: `sudo apt install ros-jazzy-geographic-msgs python3-geographiclib`

---

## Step 1 — Build the deployment bundle (simulation machine, done once)

On the **simulation machine**, from the repo root:

```bash
./deployment/build_wheels.sh
```

**The `deployment/` folder is itself the remote colcon workspace.** After building:

```txt
deployment/
  dist/
    lotusim_sdk-0.1.0-*.whl      # pure Python — agent abstractions + Bluerov2Heavy
    lotusim_client-0.1.0-*.whl   # pure Python — run_agent CLI
  src/
    lotusim_msgs/                # SOURCE package — colcon-built on the remote
    example_agent/               # reference agent package (copy it to write your own)
  setup_ros_network.sh
  README.md
```

> **Why is `lotusim_msgs` shipped as source, not as a wheel?**
> It contains compiled rosidl typesupport (`.so`) bound to the exact ROS distro
> **and** Python version it was built against. The `lotusim_msgs` package contains compiled rosidl typesupport (`.so`) bound to
> the exact ROS distro and Python version it was built against. Rebuilding from
> source against the remote's own ROS 2 Jazzy installation is the only robust
> option — and the sources are tiny.
> The pure-Python `lotusim_sdk` / `lotusim_client` have no such constraint, so
> they stay as wheels.

Copy the whole `deployment/` folder to the remote machine (USB key, scp, etc.).

---

## Step 2 — Install the wheels on the remote machine

From inside the `deployment/` folder:

```bash
source /opt/ros/jazzy/setup.bash
pip install dist/lotusim_sdk-*.whl dist/lotusim_client-*.whl

# Ensure the local pip binaries are in your PATH
export PATH=$PATH:$HOME/.local/bin
```

> **The `fault_inspection` task is self-contained.** Its YOLO detection server,
> model weights, and detection dependencies (torch / ultralytics / opencv /
> flask) all come from the `lotusim_sdk` wheel — the base `pip install` above
> pulls them automatically, so there is no extra step.

`lotusim_msgs` is built from source in the next step, alongside your agent package.

---

## Step 3 — Write your agent package and build the workspace

Your agent package lives in `src/` next to `lotusim_msgs`. Start from the provided
`src/example_agent`, or generate a fresh one:

```bash
cd src
ros2 pkg create --build-type ament_python my_agent --dependencies rclpy std_msgs lotusim_sdk
cd ..
```

Then, write your agent code. Import from `lotusim_sdk` instead of `simulation_run`.
Agents are **mission-driven**: the class stays thin and its behaviour comes from
the `missions` behaviour-tree in the JSON config (see `src/example_agent`):

```python
# my_agent/my_agent.py
from lotusim_sdk import Bluerov2Heavy


class MyBluerov(Bluerov2Heavy):
    """Behaviour is defined entirely by the scenario JSON mission tree.

    The runner calls set_missions() after instantiation; the BT tick timer then
    drives the tree. Tasks (e.g. fault_inspection) are referenced by name from
    the mission JSON and discovered via the lotusim.tasks entry-point group.
    """

    def __init__(self, sdf_string, world_name, xdyn_enabled, **kwargs):
        super().__init__(sdf_string, world_name, xdyn_enabled)
        self.renderer_type_name = "bluerov2_heavy_inspection"
```

Register it in `setup.py` under the `lotusim.agents` entry_point group:

```python
entry_points={
    "lotusim.agents": ["MyBluerov = my_agent.my_agent:MyBluerov"],
    "console_scripts": [],
}
```

Build the whole workspace (messages + your agent) in one go, from the
`deployment/` folder:

```bash
colcon build                     # builds lotusim_msgs and your agent package
source install/setup.bash        # re-source in every new shell (or add it to ~/.bashrc)
```

> Every new terminal must `source install/setup.bash` (from `deployment/`) so that
> `lotusim_msgs` and your agent are on the ROS path before running.

---

## Step 4 — Connect to the ROS 2 network

```bash
source setup_ros_network.sh
```

Verify you can see the simulation topics:

```bash
ros2 topic list   # should show /energy/mas_cmd, /energy/poses, etc.
```

If no topics appear, check that both machines are on the same LAN and
have the same `ROS_DOMAIN_ID` (edit `setup_ros_network.sh` if needed).

---

## Step 5 — Run your agent

Create a JSON config file. Spawn the agent and give it a `missions` behaviour
tree — the same mission system used host-side. This example runs the bundled
`fault_inspection` task (matches `my_config.json` in this folder):

```json
{
  "MyBluerov": {
    "spawn": { "x": 0.0, "y": 0.0, "z": -10.0, "yaw": 0.0 },
    "tick_rate_hz": 1.0,
    "missions": [
      {
        "id": "demo_mission",
        "type": "sequence",
        "children": [
          { "id": "inspection", "type": "action", "task": "fault_inspection",
            "params": { "show_window": true } }
        ]
      }
    ]
  }
}
```

`spawn` is an explicit ENU pose (same block the host scenarios use).
Alternatively use a geographic `lat`/`lon` with an `origin`/`--origin` — see
the table below. There is no `depth`/`altitude` shorthand: set the depth or
altitude directly as `spawn.z` (ENU: negative = underwater, positive = above
ground/sea level).

The `fault_inspection` task runs **on whichever machine ticks the mission** —
here, the remote. It subscribes to `/{world}/{agent}/inspection/image`
(published by the spawned model's camera plugin over ROS, not HTTP) and runs
HSV corrosion detection + YOLO crack inference locally; if `show_window` is
`true`, the OpenCV debug window opens on **this** machine. Detections are
published back as JSON on `/{world}/{agent}/inspection/detections` for Unity
(running on the simulation machine) to pick up over ROS — there is no
separate detection server or port to configure. See
[doc/MISSIONS.md](../doc/MISSIONS.md) for the full task reference.
A bare spawn (`"pose": [...]` only, no `missions`) still works for a plain agent.

Launch the agent, passing the world's geographic origin:

```bash
python3 -m lotusim_client.run_agent --world energy --config my_config.json \
    --origin 50.32879166666667 -4.195226666666667
```

> **Why `--origin` is required for a correct depth.**
> The simulation's entity manager **ignores the altitude of a `GeoPoint`** when
> spawning. To honor `depth`, a `lat`/`lon` spawn must be converted to an ENU pose
> using the world's geographic origin — exactly what the host does by reading the
> world file. The remote machine has no world file, so you pass the origin yourself.
> Read it from the world's `<spherical_coordinates>` block (`latitude_deg` /
> `longitude_deg`); for the `energy` world it is `50.32879166666667 -4.195226666666667`.
> Without `--origin`, the agent spawns at the **surface** (no trail, jerky motion).
>
> You may also put it in the config instead of the flag, either per-agent or as a
> shared top-level key (the `--origin` flag overrides both):
>
> ```json
> { "origin": [50.32879166666667, -4.195226666666667],
>   "MyBluerov": { "ais": {"patrol_file": "WP-WT1.json"}, "depth": -10.0 } }
> ```
>

The agent spawns in the Gazebo simulation on the simulation machine.
Press **Ctrl+C** to despawn the agent and exit cleanly.

---

## JSON config reference

| Key | Type | Description |
| --- | --- | --- |
| `spawn` | `{x, y, z, roll, pitch, yaw}` | Explicit ENU spawn block (same as host scenarios; any field defaults to `0`). Highest priority. Set the depth/altitude directly as `z` (negative = underwater, positive = above ground/sea level) — there is no separate `depth`/`altitude` key. |
| `poses` | list of `[x, y, z, roll, pitch, yaw]` | One entry per instance index (checked before `pose`/`lat`/`lon` if no `spawn`). |
| `pose` | `[x, y, z, roll, pitch, yaw]` | Explicit ENU spawn pose (single list form). |
| `lat` / `lon` | float | Geographic spawn position (needs `origin`/`--origin` to convert to ENU; falls back to a raw GeoPoint — which ignores altitude — without one). |
| `origin` | `[lat, lon]` | World geographic origin for geo→ENU conversion (overridden by `--origin`). |
| `missions` | array | Behaviour-tree mission spec(s); each leaf references a task by name (see `doc/MISSIONS.md`). |
| `tick_rate_hz` | float | BT mission tick rate (default: `1.0`). |
| `xdyn` | bool | Enable XDyn physics engine (default: `false`). |
| `reuse_existing` | bool | If `true` and an entity under this agent's name already exists in Gazebo, re-attach to it instead of sending a new `CREATE_CMD` (default: `false`; single-machine restart use case only). |

---

## Troubleshooting

**"Agent class not found"** — Your package is not installed or the `lotusim.agents`
entry_point is missing. Check `pip show my_agent` and re-run `colcon build`.

**"MASCmd server unavailable"** — The simulation is not running or the ROS 2 network
is not configured correctly. Run `ros2 topic list` to check connectivity.

**No topics visible** — Check `ROS_DOMAIN_ID` matches on both machines.
On managed networks, multicast may be blocked — contact your network admin.

**"Fast CDR exception" / "'Bad alloc' exception deserializing ... ParticipantEntitiesInfo"** —
Harmless. It just means the simulation machine and this remote run different ROS 2
versions, so their DDS discovery messages don't fully match. The agent still spawns
and runs normally — you can ignore these messages.
