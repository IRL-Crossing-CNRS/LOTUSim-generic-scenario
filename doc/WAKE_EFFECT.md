# Wake effect — forces and data flow

Companion to [`ARCHITECTURE.md` §9](ARCHITECTURE.md#9-wind-and-wake-agentsenvironment)
and the `wind_regions` section of [`WRITE_SCENARIO.md`](WRITE_SCENARIO.md).
Two diagrams: the force actually applied to a wind-enabled vehicle at a given
instant, and the full path a wind value takes from the Unity sliders to a
PX4-controlled drone crossing the wind farm.

---

## 1. Force diagram

Only one force formula exists in the system — `wind_regions_plugin.cpp`'s
`Update()` applies `force = mass × scaling_factor × (wind − link_velocity)` to
every wind-enabled link, every physics tick, whether that link is inside one
of `Wake`'s wake cone segments or out in the ambient wind. A wake segment
never reverses the wind vector (see the two published diagrams below) — the
force can still point upwind if the vehicle is already moving faster than the
locally reduced wind, which is a drag/deceleration effect, not a sign bug in
the segment:

```mermaid
flowchart TD
    POS["Wind-enabled link's world position (x, y)<br/>e.g. X500 base_link"]
    LOOKUP{"Inside a WindRegion shape?<br/>(box or cone segment — last matching region in the list wins)"}
    REGIONW["wind = region.linear_velocity<br/>same direction as ambient, reduced magnitude<br/>(one of Wake's chained wake cone segments)"]
    AMBIENTW["wind = ambient vector<br/>(/aerialWorld/wind, unmodified)"]
    RELV["relative velocity = wind − link_velocity"]
    SIGN{"sign of (relative velocity · wind direction)"}
    ACCEL["positive: link slower than local wind<br/>→ force pushes downwind (accelerates)"]
    DECEL["negative: link faster than local wind<br/>(already sped up by ambient before entering a weaker wake)<br/>→ force pushes upwind (decelerates)<br/>— drag, not a reversed wind vector"]
    FORCE["force = mass × scaling_factor × relative velocity"]
    APPLY["Link::AddWorldForce(force)"]

    POS --> LOOKUP
    LOOKUP -->|yes| REGIONW
    LOOKUP -->|no| AMBIENTW
    REGIONW --> RELV
    AMBIENTW --> RELV
    RELV --> SIGN
    SIGN -->|"> 0"| ACCEL
    SIGN -->|"< 0"| DECEL
    ACCEL --> FORCE
    DECEL --> FORCE
    FORCE --> APPLY
```

The mass multiplication is stock `gz-sim-wind-effects-system`'s own
approximation, kept for physical/tuning consistency with the plugin it
replaces — it cancels mass out of the resulting acceleration, so a heavy
airframe doesn't visibly under-react compared to a light one.

---

## 2. Processing diagram — sliders to a PX4 drone crossing the farm

```mermaid
flowchart TD
    subgraph UNITY["Unity (rendering only)"]
        SLIDERS["Wind sliders<br/>(hand-set ambient vector)"]
    end

    subgraph WINDAGENT["Wind agent (optional, scenario-scripted)"]
        WINDMISSION["set_wind / wait missions<br/>+ static regions (mirror_global, etc.)"]
    end

    subgraph ROS["ROS 2 topics"]
        WINDTOPIC["/aerialWorld/wind<br/>lotusim_msgs/Wind"]
        REGIONSTOPIC["/aerialWorld/wind/regions<br/>lotusim_msgs/WindRegionArray"]
        TURBINES["/{world}/wind/turbines"]
        LCOETOPIC["/{world}/lcoe"]
    end

    subgraph WAKEAGENT["Wake agent (rclpy)"]
        LARSEN["LarsenWakeModel<br/>per-turbine power"]
        BLENDED["BlendedWakeModel<br/>+ WakeRegionGenerator<br/>(chained wake cone segments)"]
        LCOECALC["LCOE accounting (optional)"]
    end

    subgraph GAZEBO["Gazebo"]
        PLUGIN["wind_regions plugin (C++)<br/>resolves wind per link,<br/>force = mass·scaling·(wind−v_link)"]
        PHYSICS["Rigid-body physics<br/>(X500 airframe)"]
    end

    subgraph PX4BLOCK["PX4 SITL"]
        PX4["Flight controller<br/>reads IMU/GPS, commands motors"]
    end

    SLIDERS -->|"ROS-TCP, ambient vector"| WINDTOPIC
    WINDMISSION -->|"while a mission is running"| WINDTOPIC
    WINDMISSION -->|"static regions, always"| REGIONSTOPIC

    WINDTOPIC --> LARSEN
    LARSEN --> TURBINES
    LARSEN --> LCOECALC
    LCOECALC --> LCOETOPIC

    WINDTOPIC --> BLENDED
    BLENDED -->|"wind_regions block only —<br/>mutually exclusive with Wind's own regions"| REGIONSTOPIC

    WINDTOPIC --> PLUGIN
    REGIONSTOPIC --> PLUGIN
    PLUGIN -->|"AddWorldForce, per wind-enabled link"| PHYSICS
    PHYSICS -->|"perturbed velocity/attitude"| PX4
    PX4 -->|"motor commands"| PHYSICS
    PHYSICS -.->|"drone moves to a new (x, y) →<br/>a different wake segment may now apply"| PLUGIN
```

Two things this diagram makes explicit:

- **`Wake` is the only place the wind is interpreted twice**, for two
  different consumers: `LarsenWakeModel` turns it into turbine power and,
  optionally, LCOE (never touches a wind-enabled vehicle); `BlendedWakeModel`
  + `WakeRegionGenerator` turn the same ambient reading into the wake cone
  segments a vehicle actually flies through. Neither model knows about PX4 or
  the drone — they only ever produce ROS messages that Gazebo's plugin later
  resolves.
- **The physics/PX4 loop (dashed edge) is the only part that is per-tick.**
  Everything upstream of `/aerialWorld/wind/regions` only recomputes when the
  ambient wind itself changes enough (`wind_changed_enough`'s hysteresis) —
  the drone can cross many wake segments between two such recomputations,
  simply by moving through an already-published, static chain of segments.
