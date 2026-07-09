import logging

import numpy as np

from lotusim_sdk.agents.environment.wind.wake_model_base import WakeModelBase

logger = logging.getLogger(__name__)


class JensenWakeModel(WakeModelBase):

    def __init__(self, diameter: float, ct: float = 0.8, air_density: float = 1.225, cp: float = 0.35, cut_in: float = 5.0, cut_out: float = 25.0, kw: float = 0.04):
        super().__init__(diameter, ct, air_density, cp, cut_in, cut_out)
        self.kw = kw

    def power(self, wind_speed: float, **kwargs) -> float:
        if wind_speed < self.cut_in or wind_speed > self.cut_out:
            return 0.0
        else:
            area = np.pi * (self.diameter / 2.0) ** 2.0
            return 0.5 * self.air_density * area * self.cp * wind_speed ** 3


    def wake_radius(self, x_dist:float)-> float:
        return self.diameter / 2 + self.kw * x_dist

    def in_wake(self, x_dist:float, lateral_dist:float)-> bool:
        r = self.wake_radius(x_dist)
        return abs(lateral_dist) < r

    def wake_deficit(self, ogWind: float, x_dist: float) -> float:
        if x_dist <= 0:
            return 0.0

        factor = (1.0 - np.sqrt(1.0 - self.ct)) / (1.0 + 2.0 * self.kw * x_dist / self.diameter) ** 2
        deficit = ogWind * factor
        return deficit

    @staticmethod
    def combined_velocity(ogWind: float, deficits) -> float:
        if not deficits:
            return ogWind
        else:
            total_deficit = np.sqrt(sum((d / ogWind) ** 2 for d in deficits))
            total_deficit = min(total_deficit, 0.999)
            return ogWind * (1.0 - total_deficit)

    def rotational_speed_rpm(
            self,
            wind_speed: float,
            yaw_factor: float = 1.0,
            tip_speed_ratio: float = 7.0
    ) -> float:
        visual_wind_speed = wind_speed * yaw_factor

        if visual_wind_speed < self.cut_in or visual_wind_speed > self.cut_out:
            return 0.0

        radius = self.diameter / 2.0
        omega = (tip_speed_ratio * visual_wind_speed) / radius
        rpm = omega * 60.0 / (2.0 * np.pi)

        return rpm

    def wind_speeds_full(self, turbines, ogWind: float, wind_vector, debug: bool = False):
        wind_xz = np.array([wind_vector[0], wind_vector[1]], dtype=float)

        if np.linalg.norm(wind_xz) == 0:
            raise ValueError("Horizontal wind vector cannot be zero.")

        w = wind_xz / np.linalg.norm(wind_xz)
        w_perp = self.perpendicular_vect_xz(w)

        turbine_facing = np.array([0.0, 1.0])
        yaw_factor = abs(np.dot(w, turbine_facing))

        turbines_sorted = sorted(
            turbines,
            key=lambda t: np.dot(np.array([t[0], t[2]]), w)
        )

        velocities = []
        rpms = []

        for i, (x_i, y_i, z_i) in enumerate(turbines_sorted):
            deficits = []

            for j in range(i):
                x_j, y_j, z_j = turbines_sorted[j]
                delta_xz = np.array([x_i - x_j, z_i - z_j])
                x_dist = np.dot(delta_xz, w)
                lateral_dist = np.dot(delta_xz, w_perp)

                if debug:
                    logger.debug("T%d→T%d: x_dist=%.1f m, lateral=%.1f m, wake_r=%.1f m",
                                 j, i, x_dist, lateral_dist, self.wake_radius(x_dist))

                if x_dist > 1e-9 and self.in_wake(x_dist, lateral_dist):
                    deficit = self.wake_deficit(ogWind, x_dist)
                    if deficit > 1e-6:
                        deficits.append(deficit)

            v_eff = self.combined_velocity(ogWind, deficits) * yaw_factor
            rpm = self.rotational_speed_rpm(v_eff)

            velocities.append(round(v_eff, 2))
            rpms.append(rpm)

            if debug:
                logger.debug("Turbine %d at (x=%.1f, z=%.1f): yaw=%.3f, v_eff=%.2f m/s, rpm=%.1f",
                             i, x_i, z_i, yaw_factor, v_eff, rpm)

        return turbines_sorted, velocities
