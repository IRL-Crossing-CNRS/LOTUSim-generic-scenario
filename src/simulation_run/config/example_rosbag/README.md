# Recording a ROS 2 bag

| File | What it does |
|---|---|
| `record_one_topic.json` | One BlueROV2 following waypoints, recording only `/energy/poses` (the ground truth of every vehicle) |

```bash
./src/simulation_run/executable/scenario_launch.sh --config example_rosbag/record_one_topic.json
ros2 bag info scenario_logs/<run>/bag
```

`"record_bag": true` records every topic instead. See [Recording a run](../../../../doc/scenario/recording.md).
