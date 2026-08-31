"""Blended wake model — spatial wake field for wind-region generation.

A distance-weighted, Ct-dependent blend of the Larsen wake and a
CFD-calibrated Gaussian, favouring Larsen in the near wake and the Gaussian
far downstream. Ported from the IRL Crossing benchmark repository
``IRL-Crossing-CNRS/lotusim-wake-models`` (``models/blended.py`` @ 8c18232),
where it was calibrated against single-turbine OpenFOAM v8/turbinesFoam CFD
at 8/10/12 m/s and validated to under 5% gradient error 1D-14D downstream —
the model that repo recommends for spatial wake fields and drone hazard
mapping, as opposed to :class:`~.larsen.LarsenWakeModel`, which is tuned for
per-turbine power.

Wraps a :class:`~.larsen.LarsenWakeModel` instance rather than building its
own — sharing it with whatever already computes farm power keeps rotor
geometry (diameter, ct, ambient_ti) as one source of truth instead of two
configs that could drift apart.

One deliberate departure from upstream, marked in place below:
:meth:`farm_velocity_at_point` projects onto the *current* wind vector before
walking turbines, where upstream's spatial queries assume wind always blows
along one fixed axis — true for the single-direction CFD runs they were
calibrated against, but not for LOTUSim's slider-driven wind.
"""

import numpy as np

from lotusim_sdk.agents.environment.wake.larsen import LarsenWakeModel


class BlendedWakeModel:
    """Larsen/Gaussian blend, in the LOTUSim ENU frame.

    Blend weight (favours Larsen near-wake, Gaussian far-wake)::

        x/D <= 1:  100% Larsen
        x/D >  1:  w(x) = clip(0.40 + 0.60*exp(-1.5*(x/D-1)/transition), 0.40, 1.0)
        transition = 1.5 * Ct / 0.75

    Calibrated Gaussian sigma, fitted to single-turbine CFD at 8/10/12 m/s::

        sigma_0(Ct)   = 0.0939*Ct + 0.3286
        sigma_inf(Ct) = 0.0875*Ct + 0.3453
        sigma(x) = sigma_inf - (sigma_inf - sigma_0) * exp(-2.796 * x/D)
    """

    SIGMA_0_SLOPE = 0.0939
    SIGMA_0_INTER = 0.3286
    SIGMA_INF_SLOPE = 0.0875
    SIGMA_INF_INTER = 0.3453
    DECAY = 2.796

    def __init__(self, larsen: LarsenWakeModel):
        self.larsen = larsen
        self.diameter = larsen.diameter
        self.ct = larsen.ct
        self.ambient_ti = larsen.ambient_ti

    def calibrated_sigma(self, x_dist: float) -> float:
        """Ct-dependent calibrated wake width sigma(x). Valid Ct 0.40-0.90."""
        x_d = x_dist / self.diameter
        sigma_0_d = self.SIGMA_0_SLOPE * self.ct + self.SIGMA_0_INTER
        sigma_inf_d = self.SIGMA_INF_SLOPE * self.ct + self.SIGMA_INF_INTER
        sigma_d = sigma_inf_d - (sigma_inf_d - sigma_0_d) * np.exp(-self.DECAY * x_d)
        return sigma_d * self.diameter

    def gaussian_velocity_at_point(self, ogWind: float, x_dist: float, r: float) -> float:
        """Calibrated-Gaussian velocity component of the blend."""
        if x_dist <= 0:
            return ogWind
        sigma = self.calibrated_sigma(x_dist)
        c_amp = 1 - np.sqrt(max(0.0, 1 - self.ct / (8 * sigma**2 / self.diameter**2)))
        deficit = ogWind * c_amp * np.exp(-(r**2) / (2 * sigma**2))
        return max(0.0, ogWind - deficit)

    def blend_weight(self, x_dist: float) -> float:
        x_d = x_dist / self.diameter
        transition = 1.5 * self.ct / 0.75
        w = 0.40 + 0.60 * np.exp(-1.5 * (x_d - 1.0) / transition)
        return float(np.clip(w, 0.40, 1.0))

    def velocity_at_point(self, ogWind: float, x_dist: float, r: float) -> float:
        """Blended velocity at (x_dist, r): w*Larsen + (1-w)*CalibratedGaussian."""
        if x_dist <= 0:
            return ogWind
        w = self.blend_weight(x_dist)
        u_l = self.larsen.velocity_at_point(ogWind, x_dist, r)
        u_g = self.gaussian_velocity_at_point(ogWind, x_dist, r)
        return w * u_l + (1 - w) * u_g

    def farm_velocity_at_point(self, ogWind: float, wind_vector, x_query: float, y_query: float, turbines) -> float:
        """Wind speed at an arbitrary ENU (x, y) point via sequential
        superposition of every upstream turbine's wake, for ``wind_vector``.

        Turbine height is not used: like upstream, this model works in the
        horizontal plane at hub height only — call it once per altitude of
        interest rather than expecting a 3D field.

        This projects turbines and the query point onto the current wind
        direction first, matching how ``LarsenWakeModel.wind_speeds_full``
        handles arbitrary direction. Upstream's own spatial queries (this
        method and ``farm_velocity_field``) assume wind always blows along
        one fixed axis instead — adequate for the single-direction CFD runs
        they were calibrated against, not for a wind vector that turns freely
        from the Unity sliders.
        """
        unit_wind = self.larsen.normalise(np.asarray(wind_vector[:2], dtype=float))
        perp = self.larsen.perpendicular_vect_xy(unit_wind)
        query_down = x_query * unit_wind[0] + y_query * unit_wind[1]
        query_cross = x_query * perp[0] + y_query * perp[1]

        upstream = []
        for tx, ty, _tz in turbines:
            t_down = tx * unit_wind[0] + ty * unit_wind[1]
            if t_down < query_down - 1e-9:
                t_cross = tx * perp[0] + ty * perp[1]
                upstream.append((t_down, t_cross))
        upstream.sort(key=lambda t: t[0])

        u_local = ogWind
        for t_down, t_cross in upstream:
            dx = query_down - t_down
            dy = query_cross - t_cross
            u_wake = self.velocity_at_point(u_local, dx, abs(dy))
            u_local = max(0.0, u_local - max(0.0, u_local - u_wake))
        return u_local
