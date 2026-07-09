import logging

import numpy as np

from lotusim_sdk.agents.environment.wind.wake_model_base import WakeModelBase

logger = logging.getLogger(__name__)


class GaussianWakeModel(WakeModelBase):

    def __init__(
        self,
        diameter: float,
        ct: float,
        air_density: float = 1.225,
        cp: float = 0.35,
        cut_in: float = 5.0,
        cut_out: float = 25.0,
        k_y: float = 0.05,
        k_z: float = 0.05,
        eps: float = 0.032,
    ):
        super().__init__(diameter, ct, air_density, cp, cut_in, cut_out)
        self.k_y = k_y
        self.k_z = k_z
        self.eps = eps

    def power(self, wind_speed: float, **kwargs) -> float:
        if wind_speed < self.cut_in or wind_speed > self.cut_out:
            return 0.0
        area = np.pi * (self.diameter / 2.0) ** 2
        return 0.5 * self.air_density * area * self.cp * wind_speed ** 3

    def gaussian_wake_deficit_full(
        self, ogWind: float, x_dist: float, y_dist: float, z_dist: float
    ) -> float:
        if x_dist <= 0:
            return 0.0

        sigma_y = self.k_y * x_dist + self.eps * self.diameter
        sigma_z = self.k_z * x_dist + self.eps * self.diameter

        if sigma_y <= 0 or sigma_z <= 0:
            return 0.0

        denom = 8.0 * sigma_y * sigma_z / (self.diameter ** 2)
        inside_sqrt = 1.0 - self.ct / denom
        if inside_sqrt <= 0:
            return 0.0

        amplitude = 1.0 - np.sqrt(inside_sqrt)
        exponent = -0.5 * ((y_dist / sigma_y) ** 2 + (z_dist / sigma_z) ** 2)
        return ogWind * amplitude * np.exp(exponent)

    def wind_speeds_full(self, turbines, ogWind: float, wind_vector, debug: bool = False):
        w = self.normalise(wind_vector)
        w_perp = self.perpendicular_vect_xz(w)

        turbines_sorted = sorted(turbines, key=lambda t: np.dot(np.array([t[0], t[2]]), w))
        velocities = []

        for i, (x_i, y_i, z_i) in enumerate(turbines_sorted):
            deficits = []
            for j in range(i):
                x_j, y_j, z_j = turbines_sorted[j]
                delta_xz = np.array([x_i - x_j, z_i - z_j])
                x_dist = np.dot(delta_xz, w)
                lateral_dist = np.dot(delta_xz, w_perp)
                vertical_dist = y_i - y_j

                deficit = self.gaussian_wake_deficit_full(
                    ogWind, x_dist, lateral_dist, vertical_dist
                )

                if debug:
                    logger.debug("T%d vs T%d: downstream=%.2f, lateral=%.2f, vertical=%.2f, deficit=%.6f",
                                 i, j, x_dist, lateral_dist, vertical_dist, deficit)

                if deficit > 1e-6:
                    deficits.append(deficit)

            velocities.append(self.combined_velocity(ogWind, deficits))

        return turbines_sorted, velocities
