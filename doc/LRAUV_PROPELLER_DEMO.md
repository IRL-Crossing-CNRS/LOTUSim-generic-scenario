# LRAUV propeller demo

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

## Manual control via ROS topic

With the scenario running, open a second terminal and enter the workspaces'
environment with `lotusim_shell`, then:

```bash
# Start the propeller RPM sequence
ros2 topic pub /energy/lrauvpropeller0/control_lrauv std_msgs/msg/Bool "data: true"

# Stop it (also sends one command at rpm=100.0, pd=0.88)
ros2 topic pub /energy/lrauvpropeller0/control_lrauv std_msgs/msg/Bool "data: false"
```

(Replace `energy` with your scenario's world name.) Tune the RPM
values sent while running in
`src/external_packages/lrauv_propeller/lrauv_propeller/lrauv_propeller.py`,
in `send_propeller_command(rpm=..., pd=...)`.

## Auto-start cycle

To make the agent automatically cycle between its `propeller_phases` (high
RPM / low RPM) on spawn instead of waiting for a manual `control_lrauv`
command, uncomment the `self.start_sequence()` call in that file's
`__init__`. Rebuild (`colcon build`) and re-source after any change.
