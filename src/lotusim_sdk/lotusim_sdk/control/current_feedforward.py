#!/usr/bin/env python3
"""Model-based current feedforward: turn a current-model prediction at the
vehicle's own depth into the body-frame force that cancels that current's drag.

Why this exists. With feedback alone, a current model changes only *what
disturbance is injected* into the simulation -- the controller never uses it.
Two current models can then differ only in how hard they happen to push the
vehicle, so a depth-resolved model has no channel through which its extra
fidelity can improve control. Feedforward is that channel: the controller
queries a current model at its own depth and pre-compensates, so a model that
resolves vertical structure can cancel a disturbance that a depth-uniform
model cannot even represent.

This also makes model-vs-model comparisons controlled: the *truth* (what
actually pushes the vehicle) can be held fixed while only the controller's
internal model changes, so any difference in tracking error or control
effort is attributable to the model, not to a different disturbance.

Sign convention: NED throughout, matching the BlueROV tasks and xdyn.

Feedforward law. xdyn applies damping on the water-relative velocity
``nu_r = nu - nu_c`` (Fossen), so the force the vehicle actually sees is

    tau_damp = -(D_lin nu_r + D_quad |nu_r| nu_r)

whereas a controller tuned in still water implicitly expects
``-(D_lin nu + D_quad |nu| nu)``. The discrepancy is the disturbance the
current introduces, and the feedforward cancels exactly it:

    tau_ff = (D_lin nu_r + D_quad |nu_r| nu_r) - (D_lin nu + D_quad |nu| nu)

At ``nu = 0`` this reduces to ``-(D_lin nu_c + D_quad |nu_c| nu_c)``: thrust
directly upstream, of the magnitude needed to hold station. The quadratic
term is kept because for the BlueROV2 Heavy it dominates the linear term
already above roughly 0.1 m/s of current (surge: 141 |u| u vs 13.7 u), well
within typical operating currents.

The current models here predict a current *field*; they are the same models
the environment side uses, evaluated on the controller side from the same
parameters, so no privileged information reaches the controller.
"""
from __future__ import annotations

import math


class NoCurrentModel:
    """Controller believes the water is still: feedforward is identically zero.

    The baseline condition -- pure feedback, i.e. the controller used in every
    run that has no feedforward at all.
    """

    name = "none"

    def current_at(self, depth_m: float) -> tuple[float, float]:
        return 0.0, 0.0


class UniformCurrentModel:
    """Depth-uniform current: one constant vector for the whole water column.

    This is what a first-order Gauss-Markov current offers a feedforward
    controller. The stochastic part of that process is zero-mean and
    unpredictable by construction, so the only component a controller can
    act on is the mean -- hence a constant. Depth does not enter, which is
    precisely the structure a depth-resolved model adds.
    """

    name = "uniform"

    def __init__(self, mean_north_ms: float, mean_east_ms: float) -> None:
        self.mean_north = float(mean_north_ms)
        self.mean_east = float(mean_east_ms)

    def current_at(self, depth_m: float) -> tuple[float, float]:
        return self.mean_north, self.mean_east


class EkmanCurrentModel:
    """Three-layer Ekman current, evaluated at a single depth.

    Port of xdyn's compiled ``ekman current`` model
    (``EkmanUWCurrentModel.cpp``: ``get_UWCurrent`` / ``getTopLayerCurrent`` /
    ``getMiddleLayerCurrent`` / ``getBottomLayerCurrent``), scalar rather than
    vectorised. Kept deliberately in step with that C++ model: the controller
    must evaluate the same current model xdyn is actually applying, or the
    feedforward compensates for the wrong physics.

    Wave height is taken as zero and the seabed depth fixed by
    ``seabed_depth_m``, matching how the ``ekman current`` environment model
    is normally configured for a BlueROV scenario.
    """

    name = "ekman"

    RHO_KG_M3 = 1026.0
    OMEGA_RAD_S = 7.2921e-5
    RHO_AIR_KG_M3 = 1.225

    def __init__(self, current_velocity_ms: float, current_orientation_deg: float,
                 top_layer_m: float, bottom_layer_m: float, u10_ms: float,
                 seabed_depth_m: float = 65.0, latitude_deg: float = 47.0,
                 wind_orientation_deg: float = 20.0) -> None:
        self.current_velocity = float(current_velocity_ms)
        self.top_layer_m = float(top_layer_m)
        self.bottom_layer_m = float(bottom_layer_m)
        self.seabed_depth_m = float(seabed_depth_m)

        theta_mid = math.radians(current_orientation_deg)
        self._mid_n = self.current_velocity * math.cos(theta_mid)
        self._mid_e = self.current_velocity * math.sin(theta_mid)

        f_and_sqrt_rho = (2 * self.OMEGA_RAD_S * math.sin(math.radians(latitude_deg))
                          * math.sqrt(self.RHO_KG_M3))
        self._sgn_f = 1.0 if f_and_sqrt_rho >= 0 else -1.0
        self._wind_angle_rad = math.radians(wind_orientation_deg)

        u10 = float(u10_ms)
        drag_coefficient = 0.79e-3 + 0.08e-3 * u10 if u10 < 20.2 else 0.002423
        wind_stress = drag_coefficient * self.RHO_AIR_KG_M3 * u10 ** 2
        # V0 is inversely proportional to the top-layer thickness -- keep the
        # coupling as-is rather than approximating it away.
        self._v0 = (math.sqrt(2) * math.pi * wind_stress
                    / (self.top_layer_m * f_and_sqrt_rho))
        self._decay = self.top_layer_m / math.pi

    def current_at(self, depth_m: float) -> tuple[float, float]:
        z = float(depth_m)
        wave_height = 0.0
        seabed = self.seabed_depth_m

        # Branch priority mirrors the C++ if/elif/elif/else chain, including
        # the factor-of-2 layer-boundary widths.
        if wave_height < z < wave_height + 2 * self.top_layer_m:
            e = math.exp(-z / self._decay)
            phase = math.pi / 4 - z / self._decay
            north = self._mid_n + self._sgn_f * self._v0 * e * math.cos(
                phase - self._sgn_f * self._wind_angle_rad)
            east = self._mid_e + self._v0 * e * math.sin(
                phase + self._sgn_f * self._wind_angle_rad)
            return north, east
        if wave_height + 2 * self.top_layer_m <= z <= seabed - 2 * self.bottom_layer_m:
            return self._mid_n, self._mid_e
        if seabed - 2 * self.bottom_layer_m < z < seabed:
            depth_factor = math.pi * (seabed - z) / self.bottom_layer_m
            e = math.exp(-depth_factor)
            c, s = math.cos(depth_factor), math.sin(depth_factor)
            north = self._mid_n * (1.0 - e * c) - self._mid_e * e * s
            east = self._mid_n * e * s + self._mid_e * (1.0 - e * c)
            return north, east
        return 0.0, 0.0


class CurrentFeedforward:
    """Current model + vehicle damping -> body-frame (surge, sway) feedforward.

    Damping coefficients are the vehicle's own and are passed in rather than
    hard-coded, keeping this class vehicle-agnostic like the rest of
    ``lotusim_sdk.control``; the BlueROV2 Heavy values live in its scenario
    parameters and come from its xdyn YAML.

    ``gain`` scales the whole feedforward. It exists to express partial trust
    in the model, and defaults to 1.0 (full compensation).
    """

    def __init__(self, model, xu: float, xuu: float, yv: float, yvv: float,
                 gain: float = 1.0, f_max: float = 100.0) -> None:
        self.model = model
        self.xu, self.xuu = float(xu), float(xuu)
        self.yv, self.yvv = float(yv), float(yvv)
        self.gain = float(gain)
        self.f_max = float(f_max)

    @staticmethod
    def _damping(v: float, lin: float, quad: float) -> float:
        return lin * v + quad * abs(v) * v

    def wrench(self, depth_m: float, psi_rad: float,
               u: float = 0.0, v: float = 0.0) -> tuple[float, float]:
        """Feedforward (surge, sway) force in the body frame, in newtons.

        ``depth_m`` positive down (NED). ``u``/``v`` are the vehicle's own
        body-frame velocities; passing them makes the compensation exact for a
        moving vehicle (transect) as well as a stationary one (station
        keeping), since the damping is quadratic and therefore not additive in
        the current.
        """
        c_n, c_e = self.model.current_at(depth_m)
        cpsi, spsi = math.cos(psi_rad), math.sin(psi_rad)
        # Current in the body frame.
        u_c = c_n * cpsi + c_e * spsi
        v_c = -c_n * spsi + c_e * cpsi

        u_r, v_r = u - u_c, v - v_c
        surge = (self._damping(u_r, self.xu, self.xuu)
                 - self._damping(u, self.xu, self.xuu))
        sway = (self._damping(v_r, self.yv, self.yvv)
                - self._damping(v, self.yv, self.yvv))

        surge = max(-self.f_max, min(self.f_max, self.gain * surge))
        sway = max(-self.f_max, min(self.f_max, self.gain * sway))
        return surge, sway


def build_feedforward(spec: dict | None, xu: float, xuu: float,
                      yv: float, yvv: float, gain: float = 1.0,
                      f_max: float = 100.0) -> CurrentFeedforward:
    """Build a feedforward from a scenario parameter block.

    ``spec`` is the ``feedforward`` params dict: ``{"model": "none"|"uniform"|
    "ekman", ...model parameters...}``. Absent or ``"none"`` yields a
    zero feedforward, so the pure-feedback baseline needs no special case at
    the call site.
    """
    spec = spec or {}
    kind = str(spec.get("model", "none")).lower()
    if kind in ("", "none"):
        model = NoCurrentModel()
    elif kind in ("uniform", "gauss", "gauss_markov"):
        model = UniformCurrentModel(spec.get("mean_x", 0.0), spec.get("mean_y", 0.0))
    elif kind == "ekman":
        model = EkmanCurrentModel(
            spec.get("current_velocity_ms", 0.0),
            spec.get("current_orientation_deg", 0.0),
            spec.get("top_layer_thickness_m", 10.0),
            spec.get("bottom_layer_thickness_m", 20.0),
            spec.get("U10_ms", 0.0),
            seabed_depth_m=spec.get("seabed_depth_m", 65.0),
            latitude_deg=spec.get("latitude_deg", 47.0),
            wind_orientation_deg=spec.get("wind_orientation_deg", 20.0))
    else:
        raise ValueError(f"unknown feedforward model {kind!r}")
    return CurrentFeedforward(model, xu, xuu, yv, yvv, gain=gain, f_max=f_max)
