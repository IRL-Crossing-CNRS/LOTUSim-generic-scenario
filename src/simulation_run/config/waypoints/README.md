# Patrol waypoint files

Data files, not scenarios. They cannot be passed to `--config`.

Each file has the shape consumed by `PatrolFileProvider`:

```json
{ "mmsi": 574821036,
  "waypoints": [ { "timestamp": "2026-08-14T06:39:14", "lat": 50.328419, "lon": -4.204740 } ] }
```

A scenario references one through a `waypoint_follower` task's
`waypoints_file` parameter. The path is resolved relative to the scenario
JSON's own directory, so scenarios in a sibling folder use `../waypoints/`:

```json
"params": { "waypoints_file": "../waypoints/waypoint_windturbine1.json" }
```

`waypoint_windturbine{1,5,9,10,13}.json` are the patrol tracks for the
correspondingly numbered turbines in the `facet_demo/` farm layout, and are
referenced by both scenarios there. `basic_examples/waypoint_solo_file.json`
reuses `waypoint_windturbine1.json`.
