# The WAM-V model

The WAM-V 16 is a 5 m twin-hull unmanned surface vehicle with two stern
engines. In LOTUSim it runs at full scale under xdyn: it floats at its
draught, rides the waves, drifts with the current, and follows a path with
the generic navigation, guidance and control pipeline. This page describes
the model, where its parameters come from, and how to drive it. It is also a
worked example of [adding a vehicle](ADD_A_VEHICLE.md).

| File | Repository |
|---|---|
| `assets/models/wamv/wamv.yaml`, `meshes/wamv_pontoons.stl` | core |
| `src/lotusim_sdk/lotusim_sdk/agents/entity/physical/wamv.py` | scenario workspace |
| `src/external_packages/wamv_gnc/` (control and allocation) | scenario workspace |
| `src/simulation_run/config/multi_vehicle_examples/wamv_*.json` | scenario workspace |

## Parameters and their sources

The physical values are those of the WAM-V 16 in the VRX simulator
([github.com/osrf/vrx](https://github.com/osrf/vrx), package `vrx_urdf`),
the public model of this boat that gives mass, inertia, damping and thruster
positions together. The comments of `wamv.yaml` give the source of each line.

| Parameter | Value | Source |
|---|---|---|
| Mass | 180 kg | `wamv_base.urdf.xacro` |
| Inertia Ixx, Iyy, Izz | 120, 393, 446 kg·m² | `wamv_base.urdf.xacro` |
| Centre of gravity | body frame origin, on the keel line | VRX `base_link` |
| Added mass | zero | VRX |
| Linear damping X, Y, Z, K, M, N | 100, 100, 500, 300, 900, 800 | VRX `SimpleHydrodynamics` |
| Quadratic damping X, Y, Z, K, M, N | 150, 100, 0, 600, 900, 800 | VRX `SimpleHydrodynamics` |
| Hull | two cylinders, radius 0.213 m, length 4.9 m, axes ±1.03 m | VRX buoyancy model |
| Engines | (−2.374, ±1.027) m, 0.191 m below the keel | VRX thruster layout |
| Thrust per engine | 250 N, forward and reverse | Set for LOTUSim: VRX and the manufacturer publish no thrust curve. Change `t_max_n` in the scenario if you have a better value |

The visual mesh (`WAM-V-Base.dae`, 5.0 × 2.49 m) and the xdyn mesh share the
same origin and axes, so the drawn hull and the simulated hull coincide.

## The hydrodynamic model

`wamv_pontoons.stl` holds the two pontoons only: the deck stays out of the
water and adds no buoyancy. The pontoons are cut every 0.35 m along their
length, because xdyn evaluates the waves at the mesh vertices; coarser
facets would average the waves out.

```yaml
external forces:
  - model: gravity
  - model: non-linear hydrostatic (exact)
  - model: non-linear Froude-Krylov
  - model: linear damping
  - model: quadratic damping
  - model: maneuvering      # wamv_prop_left,  commanded as wamv_prop_left(T)
  - model: maneuvering      # wamv_prop_right, commanded as wamv_prop_right(T)
```

| Force model | Role |
|---|---|
| `non-linear hydrostatic (exact)` | Buoyancy and restoring moments on the real hull shape. The `exact` variant keeps a twin hull steady in calm water |
| `non-linear Froude-Krylov` | The pressure of the incident waves, which makes the hull heave and pitch with them |
| `linear damping`, `quadratic damping` | Resistance to motion, and the coupling to the current: xdyn's damping acts on the velocity relative to the water |
| `maneuvering` × 2 | One per engine, a thrust `T` in newtons along the body x axis |

The launcher steps the WAM-V's xdyn server at 0.05 s (`xdyn_dt()` in
`scenario_launch.sh`), smaller than the other vehicles' 0.2 s: with no added
mass, the light hull needs the smaller step.

## Driving it

Two engines give surge and yaw; there is no sway actuator.

- **Allocation** (`wamv_allocation`, `WamvAllocationTask`) turns a surge
  force `X` and a yaw moment `N` into the two thrusts:

  ```
  T_left  = X/2 + N/(2 × 1.027)
  T_right = X/2 − N/(2 × 1.027)
  ```

  When the demand exceeds `t_max_n`, the yaw share is kept and the surge
  share reduced, so the boat keeps steering.

- **Control** (`wamv_control`, `WamvControlTask`) is the generic control task
  with three parameters the WAM-V needs:

  | Parameter | Meaning |
  |---|---|
  | `n_max_nm` | Yaw moment limit. The generic default (6 N·m, sized for the BlueROV) cannot turn this hull; `wamv_xdyn.json` uses 300 |
  | `f_max_n` | Surge force limit; `wamv_xdyn.json` uses 500 |
  | `hold_radius_m` | Holding a point without sway: outside this radius the boat points its bow at the point and drives back, inside it keeps the guidance heading |

- **Without control**, the `Wamv` class seeds both engines at zero thrust, so
  a scenario with no allocation task simply drifts.

## Scenarios

| Scenario | What it shows |
|---|---|
| `multi_vehicle_examples/wamv_drift_only.json` | No thrust: floats at its draught and drifts with the current |
| `multi_vehicle_examples/wamv_xdyn.json` | Follows a three-leg, 150 m path at 1 m/s in a JONSWAP sea (Hs 0.5 m, Tp 3 s) with line-of-sight guidance, then holds the last point |
| `multi_vehicle_examples/wamv_kinematic.json` | The same vehicle without hydrodynamics, following its commanded velocity exactly |

Spawn it at `z = −0.078`, its equilibrium draught.

## What to expect

- **Waves:** in long waves the hull follows the surface almost exactly
  (heave 0.99 of the wave amplitude in 60 m waves); in short waves it
  averages them out (0.84 at 15 m, 0.56 at 8 m), as a 5 m hull should.
- **Current:** it drifts at about 0.3 m/s in the surface Ekman current of
  `wamv.yaml`.
- **Path following** in `wamv_xdyn.json`: about 2 m from the path on
  average, more in the turns, since it cannot move sideways; the hold
  keeps it within a few metres of the last waypoint.

## Limits

- The thrust limit has no published source (see the table above).
- In perfectly calm water the hull turns very slowly on itself (about
  0.002 rad/s), from the triangulation of its mesh.
- The controller has no current feedforward: the damping it would need is not
  identified for the controller, so `xu`, `xuu`, `yv`, `yvv` stay at zero in
  `WamvControlTask`.
- No sway: in a cross current it holds a point by circling back to it.
