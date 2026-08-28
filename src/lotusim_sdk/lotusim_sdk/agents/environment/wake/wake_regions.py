"""Turns a wake field into the cone segments ``WindRegion`` can actually carry.

``lotusim_msgs/WindRegion`` supports a ``CONE_SEGMENT`` shape: a tapered
frustum, closed-form point-in-shape test (axis projection + linearly-growing
radial bound) done directly in the Gazebo plugin — no box approximation.
:class:`WakeRegionGenerator` represents each turbine's wake footprint as a
handful of these segments chained end to end (each one's ``r_end`` is the
next one's ``r_start``, so the geometry is one continuous tapered cone, not
stacked rectangles). Segment count is now purely a velocity-granularity
knob — the geometry itself is smooth within and across segments regardless
of how many there are — so a drone crossing the wake edge feels a graduated
deficit without needing anywhere near as many regions as the old box stack.

Pure Python, no ROS/rclpy dependency — the ROS-facing side (config, topic,
publishing) lives in :mod:`~.wake`.
"""

import math

import numpy as np

from lotusim_sdk.agents.environment.wake.blended import BlendedWakeModel


class WakeRegionGenerator:
    """Precomputes each turbine's wake cross-section, then projects it into
    world-frame cone segments for whatever wind direction is currently in
    effect.

    The cross-section — how wide the wake is at a given downstream distance —
    depends only on rotor geometry (diameter, ct, ambient_ti), not on wind
    speed: every calibrated deficit term in :class:`BlendedWakeModel` is
    linear in the freestream speed, so the *relative* deficit threshold used
    to find the wake's edge cancels it out. That shape is therefore computed
    once, here, in the constructor, as a chain of ``(x_start, x_end, r_start,
    r_end)`` slices — consecutive slices share a radius at their boundary, so
    the chain forms one continuous tapered cone rather than independently
    bisected, possibly mismatched, steps. :meth:`regions_for_wind` only
    rotates and translates the cached template for the wind vector on hand,
    and evaluates the actual (farm-compounded) velocity inside each segment.
    """

    def __init__(
        self,
        model: BlendedWakeModel,
        cell_diameters: float = 0.5,
        deficit_threshold: float = 0.05,
        max_downstream_diameters: float = 15.0,
    ):
        self._model = model
        self._threshold = float(deficit_threshold)
        step = float(cell_diameters) * model.diameter
        max_x = float(max_downstream_diameters) * model.diameter
        self._template = self._build_template(step, max_x)

    # ------------------------------------------------------------------
    # One-off geometry: downstream slices of (x_start, x_end, r_start, r_end),
    # evaluated at a nominal freestream speed of 1 m/s since the shape does
    # not depend on it. Chained so r_end of one slice == r_start of the
    # next, forming a continuous tapered cone instead of independent steps.
    # ------------------------------------------------------------------
    def _build_template(self, step: float, max_x: float):
        template = []
        # The first slice starts right at the rotor disk, where the
        # calibrated deficit formula is 0 by construction (x_dist<=0 guard in
        # velocity_at_point) — bisecting _boundary_radius(0) would see no
        # deficit anywhere and return a meaningless radius. Use the rotor
        # radius itself as the true starting boundary instead.
        r_prev = self._model.diameter / 2.0
        x = step
        while x <= max_x:
            centreline_deficit = 1.0 - self._model.velocity_at_point(1.0, x, 0.0)
            if centreline_deficit < self._threshold:
                break
            r_end = self._boundary_radius(x)
            template.append((x - step, x, r_prev, r_end))
            r_prev = r_end
            x += step
        return template

    def _boundary_radius(self, x_dist: float) -> float:
        """Smallest ``r`` at ``x_dist`` where the relative deficit has fallen
        below :attr:`_threshold` — found by bisection, since deficit is
        monotonically non-increasing in ``r``."""
        lo, hi = 0.0, max(self._model.larsen.wake_radius(x_dist), self._model.diameter)
        while (1.0 - self._model.velocity_at_point(1.0, x_dist, hi)) > self._threshold:
            hi *= 1.5
            if hi > 50.0 * self._model.diameter:
                break
        for _ in range(25):
            mid = 0.5 * (lo + hi)
            deficit = 1.0 - self._model.velocity_at_point(1.0, x_dist, mid)
            if deficit > self._threshold:
                lo = mid
            else:
                hi = mid
        return hi

    # ------------------------------------------------------------------
    # Per-update: project the cached template into world-frame cone segments.
    # ------------------------------------------------------------------
    def regions_for_wind(self, turbines_named, wind_vector):
        """Return a list of region descriptors for the current wind.

        ``turbines_named`` is ``[(name, x, y, z), ...]`` in ENU; ``wind_vector``
        is ``[vx, vy]``. Each descriptor is a plain dict — ``{"id", "origin_x",
        "origin_y", "origin_z", "axis_x", "axis_y", "length", "r_start",
        "r_end", "vx", "vy", "vz"}`` — so this module stays free of any ROS
        message dependency.

        ``origin_z`` is the turbine's hub altitude: the wake is a horizontal
        tube centred on the rotor, and the consumer measures its radius
        perpendicular to the (horizontal) axis, vertical offset included.
        """
        if not self._template:
            return []

        wind_xy = np.asarray(wind_vector[:2], dtype=float)
        og_speed = float(np.linalg.norm(wind_xy))
        if og_speed < 1e-6:
            return []
        unit_wind = wind_xy / og_speed

        turbines = [(x, y, z) for _name, x, y, z in turbines_named]
        regions = []

        for name, tx, ty, tz in turbines_named:
            origin = np.array([tx, ty])
            for i, (x_start, x_end, r_start, r_end) in enumerate(self._template):
                centre = origin + 0.5 * (x_start + x_end) * unit_wind
                speed = self._model.farm_velocity_at_point(
                    og_speed, wind_vector, centre[0], centre[1], turbines
                )
                if (og_speed - speed) / og_speed < self._threshold:
                    # Compounding from other turbines may have already
                    # recovered this slice even where this turbine's own
                    # template says it shouldn't have — skip it rather than
                    # publish a segment with no real deficit in it.
                    continue

                segment_origin = origin + x_start * unit_wind
                vx, vy = unit_wind * speed

                regions.append({
                    "id": f"wake_{name}_{i}",
                    "origin_x": float(segment_origin[0]), "origin_y": float(segment_origin[1]),
                    "origin_z": float(tz),
                    "axis_x": float(unit_wind[0]), "axis_y": float(unit_wind[1]),
                    "length": float(x_end - x_start),
                    "r_start": float(r_start), "r_end": float(r_end),
                    "vx": float(vx), "vy": float(vy), "vz": 0.0,
                })

        return regions


def wind_changed_enough(
    last_wind_vector,
    new_wind_vector,
    direction_hysteresis_deg: float,
    speed_hysteresis_mps: float,
) -> bool:
    """True if ``new_wind_vector`` has drifted far enough from
    ``last_wind_vector`` to be worth regenerating and republishing wake
    regions.

    Regenerating on every wind message would mean a 10 Hz ``Wind`` publish
    rate driving a 10 Hz reshuffle of every wake box on a topic the Gazebo
    plugin scans per wind-enabled link, per physics tick (500 Hz) — wasted
    work for a wind vector that barely moved. ``last_wind_vector`` of
    ``None`` (nothing published yet) always triggers a regeneration.
    """
    if last_wind_vector is None:
        return True

    lx, ly = last_wind_vector
    nx, ny = new_wind_vector
    last_speed = math.hypot(lx, ly)
    new_speed = math.hypot(nx, ny)

    if abs(new_speed - last_speed) >= speed_hysteresis_mps:
        return True
    if last_speed < 1e-6 or new_speed < 1e-6:
        # One of the two is calm: any nonzero-vs-zero transition is a change
        # of substance (regions appear/disappear), direction is meaningless.
        return (last_speed < 1e-6) != (new_speed < 1e-6)

    cos_angle = (lx * nx + ly * ny) / (last_speed * new_speed)
    cos_angle = max(-1.0, min(1.0, cos_angle))
    angle_deg = math.degrees(math.acos(cos_angle))
    return angle_deg >= direction_hysteresis_deg
