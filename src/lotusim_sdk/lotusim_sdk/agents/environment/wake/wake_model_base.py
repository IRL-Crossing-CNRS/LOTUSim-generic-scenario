from abc import ABC, abstractmethod

import numpy as np


class WakeModelBase(ABC):
    """Turbine and inflow parameters shared by every wake model.

    Positions and wind vectors are in the LOTUSim ENU frame throughout: a
    turbine is ``(x, y, z)`` with ``x``/``y`` horizontal and ``z`` the hub
    height, a wind vector is ``[vx, vy]`` in that same horizontal plane.
    """

    def __init__(
        self,
        diameter: float,
        ct: float,
        air_density: float = 1.225,
        cp: float = 0.35,
        cut_in: float = 5.0,
        cut_out: float = 25.0,
        ambient_ti: float = 0.08,
        shear_exponent: float = 0.12,
    ):
        self.diameter = float(diameter)
        self.ct = float(ct)
        self.air_density = float(air_density)
        self.cp = float(cp)
        self.cut_in = float(cut_in)
        self.cut_out = float(cut_out)
        self.ambient_ti = float(ambient_ti)
        self.shear_exponent = float(shear_exponent)

    @abstractmethod
    def power(self, wind_speed: float, **kwargs) -> float:
        """Return electrical power [W] for the given effective wind speed."""

    @abstractmethod
    def wind_speeds_full(self, turbines, wind_vector, debug: bool = False) -> tuple:
        """Return ``(turbines_sorted, velocities, rpms)`` after wake losses.

        ``turbines_sorted`` is the layout reordered upstream to downstream
        along ``wind_vector``; ``velocities`` and ``rpms`` follow that order.
        The freestream speed is the norm of ``wind_vector`` — callers do not
        pass it separately.
        """

    @staticmethod
    def normalise(vector):
        vector = np.array(vector, dtype=float)
        magnitude = np.linalg.norm(vector)
        if magnitude == 0:
            raise ValueError("Wind vector cannot be zero.")
        return vector / magnitude

    @staticmethod
    def perpendicular_vect_xy(wind_unit_vector_xy):
        return np.array([-wind_unit_vector_xy[1], wind_unit_vector_xy[0]])
