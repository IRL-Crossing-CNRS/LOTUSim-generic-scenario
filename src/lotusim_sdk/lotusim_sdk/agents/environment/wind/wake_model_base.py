from abc import ABC, abstractmethod

import numpy as np


class WakeModelBase(ABC):

    def __init__(
        self,
        diameter: float,
        ct: float,
        air_density: float = 1.225,
        cp: float = 0.35,
        cut_in: float = 5.0,
        cut_out: float = 25.0,
    ):
        self.diameter = diameter
        self.ct = ct
        self.air_density = air_density
        self.cp = cp
        self.cut_in = cut_in
        self.cut_out = cut_out

    @abstractmethod
    def power(self, wind_speed: float, **kwargs) -> float:
        """Return electrical power [W] for the given effective wind speed."""

    @abstractmethod
    def wind_speeds_full(
        self, turbines, ogWind: float, wind_vector, debug: bool = False
    ) -> tuple:
        """Return (turbines_sorted, effective_velocities) after applying wake losses."""

    @staticmethod
    def normalise(vector):
        vector = np.array(vector, dtype=float)
        magnitude = np.linalg.norm(vector)
        if magnitude == 0:
            raise ValueError("Wind vector cannot be zero.")
        return vector / magnitude

    @staticmethod
    def perpendicular_vect_xz(wind_unit_vector_xz):
        return np.array([-wind_unit_vector_xz[1], wind_unit_vector_xz[0]])

    @staticmethod
    def wind_speed(wind_vector_multi):
        u_inf = []
        for wind_vector in wind_vector_multi:
            speed = np.sqrt(wind_vector[0] ** 2 + wind_vector[1] ** 2)
            u_inf.append(float(f"{speed:.2f}"))
        return u_inf

    @staticmethod
    def combined_velocity(ogWind: float, deficits) -> float:
        if not deficits:
            return ogWind
        total_deficit = np.sqrt(sum((d / ogWind) ** 2 for d in deficits))
        total_deficit = min(total_deficit, 0.999)
        return ogWind * (1.0 - total_deficit)
