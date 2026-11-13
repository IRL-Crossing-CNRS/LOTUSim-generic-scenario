# Generic scenario reference

A generic scenario is a JSON file, passed to `scenario_launch.sh --config`.
These pages describe everything it can contain.

| Page | What it covers |
|---|---|
| [The scenario file](scenario_file.md) | A minimal scenario, every top-level key, every agent key, spawn poses |
| [Missions and behaviour trees](missions.md) | How an agent's `missions` run: sequences, parallels, leaves, examples |
| [Task reference](tasks.md) | Every task usable in a mission, with all its parameters |
| [Ocean current](current.md) | Currents for xdyn vessels and for Kinematic vehicles |
| [Wind and wake](wind.md) | The ambient wind, wind regions, turbine wakes and the farm's power |
| [Waves and weather](sea.md) | One sea for every vessel and the renderer: a sea state, a Beaufort force, or the weather of a place and date |
| [Recording a run](recording.md) | CSV telemetry of every agent, ROS 2 bag of the sensor streams |
| [Agents on a remote machine](remote.md) | What changes in the file when the agents run on another machine |

Ready-to-run scenarios are in `src/simulation_run/config/`, described in its
[README](../../src/simulation_run/config/README.md); the launcher's options
are in [LAUNCH_OPTIONS.md](../LAUNCH_OPTIONS.md).

To add a task or a vehicle class of your own, see the
[developer guide](../DEVELOPER.md) and [ADD_A_VEHICLE.md](../ADD_A_VEHICLE.md).
