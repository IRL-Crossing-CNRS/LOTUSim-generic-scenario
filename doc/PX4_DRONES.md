# PX4 drones

An `X500` drone can fly two ways:

| | PX4 (`"px4": true`) | LOTUSim guidance (default) |
|---|---|---|
| Who flies it | PX4 SITL, the real autopilot software, on simulated sensors and motors | LOTUSim's guidance and control tasks, moving the model directly |
| Commanded from | QGroundControl, or the `px4_offboard_patrol` task over MAVLink | the scenario's mission tasks |
| Arming, flight modes, failsafes | PX4's own | none |
| Needs | a built PX4-Autopilot checkout | nothing more |
| Use it for | behaviour as close as possible to a real PX4 drone | many drones in a scenario at low cost |

This page is about the first. PX4 is a separate checkout you build once; it
attaches to the drone LOTUSim spawns in the aerial world. How the two are
wired is in [ARCHITECTURE.md §5.1](ARCHITECTURE.md).

## 1. Install PX4-Autopilot

The installer builds it when you answer yes to its PX4 question; the manual
build, and how much disk it takes, are in
[How_To_Install_LOTUSim — PX4 SITL](https://github.com/IRL-Crossing-CNRS/How_To_Install_LOTUSim#optional-px4-sitl-aerial-drones).

## 2. Point LOTUSim at your PX4 checkout

LOTUSim looks for a built PX4 checkout at `$PX4_AUTOPILOT_PATH`, by default
`~/PX4-Autopilot`. If you cloned it elsewhere, export its path before
launching:

```bash
export PX4_AUTOPILOT_PATH=/path/to/PX4-Autopilot
```

## 3. Enable PX4 on an X500 agent

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

The aerial world and every PX4 run on reserved cores: the last two of the
machine by default, or one per PX4 drone, up to half of the machine, for a
scenario with more than two. Set `PX4_CPU_AFFINITY` (a `taskset -c` list) to
choose them yourself, or to an empty string for no pinning.

Per-instance PX4 console output lands in
`scenario_logs/<timestamp>/px4_sitl_<agent_name>.log` — check there first if
a PX4 drone isn't responding. A clean attach looks like `gazebo already
running world: aerialWorld` / `PX4_GZ_MODEL_NAME set, PX4 will attach to
existing model`, followed by MAVLink listening on UDP port 14550.

## 4. Flying it: manual or offboard

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
| `start_time_s` | `0.0` | Simulation time at which to arm. PX4 becomes ready on the wall clock, so repeats of a scenario take off at different times unless this holds them to the same one; leave it about 15 s to become ready and, for repeats that match, about 60 s for the height estimate to settle. |
| `max_speed_m_s` | PX4's `MPC_XY_VEL_MAX`, 12 m/s | Horizontal speed limit. With position setpoints PX4 flies at the speed its position gain asks for, far too fast for legs of a few metres. |
| `setpoint_rate_hz` | `10.0` | Offboard setpoint stream rate. PX4 leaves OFFBOARD mode if the stream drops below 2 Hz. |

`facet_demo/demo_facet_october2026.json` flies its whole fleet this way: its
15 `X500_inspection` drones are PX4 drones, one PX4 process each, so the wind
and the turbine wakes push them. They patrol at 1 m/s (`max_speed_m_s`). The
drone that watches the crack hovers on a single waypoint with `loop: true` and
`hold_radius_m: 0`, so that the setpoints never stop: PX4 leaves OFFBOARD when
they do. The drones spawn with `"sdf_file": "model-battery.sdf"`, the PX4 model
with a LOTUSim battery sensor, which drains at a fixed 200 W since PX4's motor
commands do not reach it.

`px4_control` is descriptive only; the attached task is what drives the
vehicle. Keep the two consistent, as setting `"manual"` while attaching
`px4_offboard_patrol` (or the reverse) has no defined behaviour.

### QGroundControl

Install it as described in
[How_To_Install_LOTUSim — QGroundControl](https://github.com/IRL-Crossing-CNRS/How_To_Install_LOTUSim#qgroundcontrol),
then start it (`~/Applications/QGroundControl-x86_64.AppImage`).
QGroundControl connects automatically to any PX4 SITL instance listening on
`localhost:14550`. With the scenario running it detects the vehicle within a
few seconds and shows live telemetry. Arm and command takeoff or goto from
its flight controls.
