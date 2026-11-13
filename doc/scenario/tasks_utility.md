# Wait and wind tasks

Tasks that act on the mission's timing or on the world.

## `set_wind` → `SetWindTask`

Sets the ambient wind vector, then returns `SUCCESS`. Unlike every other task
here, its host is not a vehicle: it only runs on a `Wind` **environment** agent
([Wind and wake](wind.md)), which is what makes a scripted wind history expressible as a plain
`sequence`.

| Param | Type | Default | Notes |
|---|---|---|---|
| `x` / `y` / `z` | float (m/s) | current value | Wind velocity in the world ENU frame (x=East, y=North, z=Up). Omitted components are left untouched. |

Publishes `/aerialWorld/wind` (`lotusim_msgs/Wind`, `TRANSIENT_LOCAL`) through
its host agent — see [Wind and wake](wind.md) for who else reads and writes that topic.

## `wait` → `WaitTask`

Stays `RUNNING` for `duration_s`, then returns `SUCCESS`. Any agent — it reads
and changes nothing but its own progress, so it drops safely into any
`sequence`/`parallel` next to any other task.

| Param | Type | Default | Notes |
|---|---|---|---|
| `duration_s` | float (s) | `0.0` | Wall-clock — no node in this codebase sets `use_sim_time`, so this holds regardless of Gazebo's `real_time_factor`. |

Mainly useful to space out a run of instantly-succeeding actions — a `Sequence`
advances through every immediately-succeeding child within the same tick (see
`lotusim_sdk/bt/composites.py`), so e.g. two `set_wind` back to back apply
instantly and only the second is ever actually observed. Interleave a `wait`:
