"""Larsen wake model — per-turbine power, farm energy yield and LCOE.

Reference: Larsen, G.C. (2009). *A simple stationary semi-analytical wake
model*. Risoe-R-1713(EN), DTU Wind Energy.

Ported from the IRL Crossing benchmark repository
``IRL-Crossing-CNRS/lotusim-wake-models`` (``models/larsen.py`` @ 8c18232),
which validated it against OpenFOAM v8 + turbinesFoam actuator-line CFD and
FLORIS v4 on the NREL 5MW reference turbine. It is the best of the four
benchmarked models on every power protocol — P2-1 RMSE 0.212 MW against
0.309 MW for Jensen and 0.767 MW for FLORIS-Gauss — and is the only wake
model this package provides.

Two deliberate departures from the upstream code, both marked in place below:

* **Coordinates.** Upstream lays turbines out as ``(x_lateral, y_hub_height,
  z_downstream)`` — its *y* is the vertical axis. This port is written
  natively in the ENU frame every other position in LOTUSim uses (``x``/``y``
  horizontal, ``z`` up), so no caller has to remap axes and the wake model
  cannot silently disagree with the ``wind_regions`` Gazebo plugin about which
  axis is up. The maths is unchanged: only which column of the turbine array
  is read as height.
* **Yaw.** Upstream assumes turbines bolted facing north, so any wind without
  a northward component produces zero power across the whole farm. See
  :meth:`_project_coordinates`.

The upstream spatial helpers (``velocity_gradient_at_point``,
``wake_hazard_zone``, ``farm_velocity_at_point``, ``ti_at_point``) are
deliberately *not* ported: wake *fields* and drone hazard zones are the job of
upstream's ``BlendedWakeModel``, and nothing here consumes them yet. Only the
power path lives in this module.
"""

import logging

import numpy as np

from lotusim_sdk.agents.environment.wake.wake_model_base import WakeModelBase

logger = logging.getLogger(__name__)


class LarsenWakeModel(WakeModelBase):
    """Semi-analytical Larsen wake, in the LOTUSim ENU frame.

    Turbines are ``(x, y, z)`` with ``x``/``y`` horizontal and ``z`` the hub
    height; the wind vector is ``[vx, vy]`` in that same horizontal plane.
    """

    def __init__(
        self,
        diameter: float,
        ct: float = 0.8,
        air_density: float = 1.225,
        cp: float = 0.35,
        cut_in: float = 5.0,
        cut_out: float = 25.0,
        ambient_ti: float = 0.08,
        shear_exponent: float = 0.12,
        tip_speed_ratio: float = 7.0,
    ):
        super().__init__(
            diameter,
            ct,
            air_density,
            cp,
            cut_in,
            cut_out,
            ambient_ti=ambient_ti,
            shear_exponent=shear_exponent,
        )
        self.tip_speed_ratio = float(tip_speed_ratio)
        self.radius = self.diameter / 2.0
        self.area = np.pi * self.radius**2

        # Wake-geometry constants: pure functions of (ct, diameter,
        # ambient_ti), so they are resolved once here rather than on every
        # turbine pair of every wind message. Computing them now also means an
        # unusable rotor geometry raises while the scenario is still starting
        # up, instead of on the first wind message minutes later.
        self._m_const = 1.0 / np.sqrt(1.0 - self.ct)
        self._k_const = np.sqrt((self._m_const + 1.0) / 2.0)
        self._r96_const = self._compute_r96()
        self._x0_const = self._compute_x0()
        self._c1_const = self._compute_c1()

    # ------------------------------------------------------------------
    # Wake geometry (Larsen 2009, section 3)
    # ------------------------------------------------------------------
    def _compute_r96(self) -> float:
        """Empirical wake radius at 9.6 rotor diameters downstream."""
        a1, a2, a3, a4 = 0.435449861, 0.797853685, -0.124807893, 0.136821858
        b1 = 9.5
        return a1 * np.exp(a2 * self.ct**2 + a3 * self.ct + a4) * (b1 * self.ambient_ti + 1.0) * self.diameter

    def _compute_x0(self) -> float:
        """Virtual wake origin, upstream of the rotor."""
        denom = (2.0 * self._r96_const / (self._k_const * self.diameter)) ** 3 - 1.0
        if np.isclose(denom, 0.0):
            raise ValueError(
                "Invalid denominator in the Larsen x0 calculation — the rotor "
                f"geometry (diameter={self.diameter}, ct={self.ct}, "
                f"ambient_ti={self.ambient_ti}) has no usable virtual wake origin."
            )
        return 9.6 * self.diameter / denom

    def _compute_c1(self) -> float:
        """Prandtl mixing-length constant of the wake profile."""
        return (
            ((self._k_const * self.diameter / 2.0) ** (5.0 / 2.0))
            * ((105.0 / (2.0 * np.pi)) ** (-1.0 / 2.0))
            * ((self.ct * self.area * self._x0_const) ** (-5.0 / 6.0))
        )

    def wake_radius(self, x_dist: float) -> float:
        """Wake radius [m] at ``x_dist`` metres downstream of a rotor."""
        if x_dist <= 0:
            return self.radius
        return ((105.0 * self._c1_const**2) / (2.0 * np.pi)) ** (1.0 / 5.0) * (
            self.ct * self.area * (x_dist + self._x0_const)
        ) ** (1.0 / 3.0)

    def local_ti(self, x_dist: float) -> float:
        """Turbulence intensity in the wake — Crespo & Hernandez (1996)."""
        if x_dist <= 0:
            return self.ambient_ti
        x_d = x_dist / self.diameter
        return self.ambient_ti + 0.5 * (self.ct * self.ambient_ti) ** 0.25 * x_d ** (-0.32)

    def wake_centreline_offset(self, x_dist: float, yaw_angle_rad: float) -> float:
        """Lateral deflection [m] of the wake centreline — Jimenez (2009)."""
        if abs(yaw_angle_rad) < 1e-6:
            return 0.0
        return (self.ct / 2.0) * np.sin(yaw_angle_rad) * np.cos(yaw_angle_rad) ** 2 * x_dist

    def meandering_factor(self, x_dist: float, r: float) -> float:
        """Dynamic wake meandering correction — Larsen et al. (2008).

        A wake that wanders laterally spends only part of its time on any given
        downstream point, so the deficit that point actually sees is smaller
        than the steady-state one. Bounded to [0.7, 1.0].
        """
        r_wake = self.wake_radius(x_dist)
        if r_wake <= 0:
            return 1.0
        sigma_m = 0.5 * r_wake
        weight = np.exp(-0.5 * (r / (sigma_m + 1e-6)) ** 2)
        return float(np.clip(0.7 + 0.3 * weight, 0.7, 1.0))

    def velocity_at_point(self, ogWind: float, x_dist: float, r: float) -> float:
        """Wake velocity [m/s] ``x_dist`` downstream, ``r`` off the centreline.

        The centreline deficit is the CFD-calibrated fit ``0.58 * (x/D)^-0.35``
        rather than Larsen's analytical one — fitted upstream on the NREL 5MW
        (D=126 m) at 8/10/12 m/s. The Larsen solution supplies the radial
        profile shape, the fit supplies its amplitude.
        """
        if x_dist <= 0:
            return ogWind
        if r >= self.wake_radius(x_dist):
            return ogWind

        x_eff = x_dist + self._x0_const
        rhs = (35.0 / (2.0 * np.pi)) ** (3.0 / 10.0) * (3.0 * self._c1_const**2) ** (-1.0 / 5.0)
        if r < 1e-6:
            bracket = -rhs
        else:
            bracket = r**1.5 * (3.0 * self._c1_const**2 * self.ct * self.area * x_eff) ** (-0.5) - rhs
        profile = bracket**2 / rhs**2

        x_d = x_dist / self.diameter
        deficit_centreline = ogWind * 0.58 * x_d ** (-0.35)
        # Higher local turbulence mixes the wake out faster, so the deficit a
        # downstream rotor sees shrinks as added turbulence grows.
        ti_scale = (self.ambient_ti / self.local_ti(x_dist)) ** 0.5
        deficit_centreline *= ti_scale

        return max(0.0, ogWind - deficit_centreline * profile)

    # ------------------------------------------------------------------
    # Power
    # ------------------------------------------------------------------
    def shear_adjusted_speed(self, ogWind: float, z: float, hub_height: float) -> float:
        """Power-law wind shear profile — IEC 61400-1 Ed.3."""
        return ogWind * (z / hub_height) ** self.shear_exponent

    def rotor_averaged_speed(self, ogWind: float, hub_height: float, n_points: int = 20) -> float:
        """Area-weighted rotor-disk average of the sheared inflow.

        Honrubia et al. (2012). Wind shear means the blade tips do not see the
        hub-height speed; the chord-weighted average over the disk is what the
        rotor actually extracts from.
        """
        z_samples = np.linspace(hub_height - self.radius, hub_height + self.radius, n_points)
        # A rotor whose disk reaches the ground (hub_height <= radius) would
        # otherwise raise a negative base to a fractional power and yield NaN
        # power for the whole farm. Upstream never guards this because its
        # reference turbine (D=126 m, hub 90 m) cannot reach it.
        z_samples = np.maximum(z_samples, 1e-6)
        weights = np.sqrt(np.maximum(0.0, self.radius**2 - (z_samples - hub_height) ** 2))
        u_samples = np.array([self.shear_adjusted_speed(ogWind, z, hub_height) for z in z_samples])
        if weights.sum() > 0:
            return float(np.average(u_samples, weights=weights))
        return ogWind

    def power(self, wind_speed: float, hub_height: float = 90.0, **kwargs) -> float:
        """Electrical power [W] for an effective inflow speed at ``hub_height``.

        Note there is no rated-power ceiling: this follows the reference
        implementation, so the curve stays cubic all the way to ``cut_out``.
        """
        if wind_speed < self.cut_in or wind_speed > self.cut_out:
            return 0.0
        u_rotor = self.rotor_averaged_speed(wind_speed, hub_height)
        return 0.5 * self.air_density * self.area * self.cp * u_rotor**3

    def rotational_speed_rpm(self, wind_speed: float) -> float:
        """Rotor speed [rpm] at constant tip-speed ratio."""
        if wind_speed < self.cut_in or wind_speed >= self.cut_out:
            return 0.0
        omega = (self.tip_speed_ratio * wind_speed) / self.radius
        return omega * 60.0 / (2.0 * np.pi)

    # ------------------------------------------------------------------
    # Farm solution
    # ------------------------------------------------------------------
    def _project_coordinates(self, turbines: np.ndarray, wind_vector):
        """Project an ENU turbine layout onto the wind direction.

        Returns ``(downstream, crosswind, height, ogWind, yaw_factor)``, the
        first three being per-turbine arrays in metres.

        ``yaw_factor`` derates power by how far the wind is off the turbines'
        facing axis (+y, North). We take ``abs`` of that projection where
        upstream takes ``max(0, ...)``: upstream's turbines are bolted facing
        north, so a southerly wind zeroes its entire farm. Since LOTUSim's wind
        direction is driven freely from the Unity sliders, that asymmetry would
        make half the compass produce no power at all. This matches the
        behaviour of the Jensen model this one replaces.
        """
        turbines = np.asarray(turbines, dtype=float)
        if turbines.ndim != 2 or turbines.shape[1] != 3:
            raise ValueError("turbines must be shape (n_turbines, 3), as (x, y, z) ENU.")

        wind_xy = np.array([wind_vector[0], wind_vector[1]], dtype=float)
        unit_wind = self.normalise(wind_xy)
        ogWind = float(np.linalg.norm(wind_xy))
        perp = self.perpendicular_vect_xy(unit_wind)

        horizontal = turbines[:, [0, 1]]
        height = turbines[:, 2]
        downstream = horizontal @ unit_wind
        crosswind = horizontal @ perp

        turbine_facing = np.array([0.0, 1.0])
        yaw_factor = abs(float(np.dot(unit_wind, turbine_facing)))

        return downstream, crosswind, height, ogWind, yaw_factor

    def wind_speeds_full(self, turbines, wind_vector, debug: bool = False):
        """Effective inflow speed at every turbine, upstream to downstream.

        Returns ``(turbines_sorted, velocities, rpms)``. Wakes are combined by
        sequential *local* superposition — each upstream rotor's deficit is
        applied to the speed already reduced by the rotors ahead of it, in
        downstream order — rather than by the root-sum-square of freestream
        deficits the Jensen/Gaussian models used. That ordering is what lets
        deep rows keep losing energy instead of saturating, and is the main
        reason this model tracks CFD on the 4x4 layout (P2-3 RMSE 0.284 MW vs
        0.547 MW for Jensen).
        """
        turbines_arr = np.asarray(turbines, dtype=float)
        projection = self._project_coordinates(turbines_arr, wind_vector)
        downstream, _, height, _, yaw_factor = projection

        order = np.argsort(downstream)
        turbines_sorted = turbines_arr[order].tolist()
        velocities = []
        rpms = []

        for idx in order:
            v_eff = self._effective_speed_at_turbine(idx, projection, wind_vector)
            velocities.append(round(float(v_eff), 2))
            rpms.append(round(self.rotational_speed_rpm(v_eff), 1))
            if debug:
                x, y, z = turbines_arr[idx]
                logger.debug(
                    "Turbine at (x=%.1f, y=%.1f, z=%.1f): yaw=%.3f, v_eff=%.2f m/s",
                    x,
                    y,
                    z,
                    yaw_factor,
                    v_eff,
                )

        return turbines_sorted, velocities, rpms

    def _effective_speed_at_turbine(self, turbine_index: int, projection, wind_vector) -> float:
        """Inflow speed at one turbine, after every rotor upstream of it."""
        downstream, crosswind, height, ogWind, yaw_factor = projection

        target_down = downstream[turbine_index]
        target_cross = crosswind[turbine_index]
        target_height = height[turbine_index]

        upstream_indices = [
            j for j in range(len(downstream)) if j != turbine_index and downstream[j] < target_down - 1e-9
        ]
        upstream_indices.sort(key=lambda j: downstream[j])

        yaw_angle = np.arctan2(wind_vector[0], wind_vector[1])
        u_local = ogWind

        for j in upstream_indices:
            dx = target_down - downstream[j]
            dy = target_cross - crosswind[j]
            dh = target_height - height[j]
            r = np.sqrt(dy**2 + dh**2)
            if abs(r) > self.wake_radius(dx):
                continue

            wake_offset = self.wake_centreline_offset(dx, yaw_angle)
            effective_r = np.sqrt((dy - wake_offset) ** 2 + dh**2)

            # Wake expansion rate k as a function of turbulence — the deficit
            # is damped by how much faster the wake spreads under the locally
            # elevated turbulence than under ambient conditions.
            k_amb = 0.38 * self.ambient_ti + 0.004
            k_local = 0.38 * self.local_ti(dx) + 0.004
            ti_ratio = (k_amb / k_local) ** 0.5

            meander = self.meandering_factor(dx, effective_r)
            u_wake = self.velocity_at_point(u_local, dx, effective_r)
            deficit = max(0.0, u_local - u_wake) * ti_ratio * meander

            # Applied twice, deliberately. Upstream does the same (models/
            # larsen.py:313-314 @ 8c18232) and that doubled deficit is baked
            # into the CFD-validated results this model is trusted for — the
            # published [3.724, 1.707, 0.858] MW for the 3-turbine 7D layout
            # only come out with both subtractions. Removing one silently
            # under-predicts wake losses and breaks agreement with those
            # reference results, so it stays until upstream decides otherwise.
            u_local = max(0.0, u_local - deficit)
            u_local = max(0.0, u_local - deficit)

        return u_local * yaw_factor
