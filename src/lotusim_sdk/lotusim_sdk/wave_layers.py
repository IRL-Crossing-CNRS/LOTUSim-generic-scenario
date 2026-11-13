"""The waves of a weather that changes during the run, as fading layers.

xdyn evaluates the waves (its Airy model, from the rays file every vehicle model
reads); this module only keeps the list of components and the fade of each one,
for the rays file xdyn re-reads and the message the renderer draws:

- a change of sea state does not cut the old sea: its layer fades out while the
  new one fades in, over ``ramp_s`` seconds of simulation time;
- each component carries its layer's fade (``fade_in_start``, ``fade_in_duration``,
  ``fade_out_start``, ``fade_out_duration``); xdyn and the renderer weight it by
  sqrt(u_in (1 - u_out)), so the variance of the surface moves linearly from the
  old sea to the new one (the phases of the two layers are independent).
"""

from __future__ import annotations

import threading
from dataclasses import dataclass
from typing import Dict, List, Optional

from lotusim_sdk.waves import WaveRays

#: A fade that never starts.
NEVER = 1e12

FADE_KEYS = ("fade_in_start", "fade_in_duration", "fade_out_start", "fade_out_duration")


@dataclass
class _Layer:
    rays: WaveRays
    fade_in_start: float
    fade_in_duration: float
    fade_out_start: float = NEVER
    fade_out_duration: float = 0.0

    def faded_out(self, t: float) -> bool:
        return t >= self.fade_out_start + self.fade_out_duration


class WaveField:
    """The current sea, as fading layers of wave components. Thread-safe."""

    def __init__(self, rays: Optional[WaveRays] = None):
        self._layers: List[_Layer] = []
        self._lock = threading.Lock()
        #: Latest simulation time seen (from the poses the physics publishes).
        self.last_t = 0.0
        if rays is not None and len(rays):
            self._layers.append(_Layer(rays, -NEVER, 0.0))

    def change(self, rays: WaveRays, t: float, ramp_s: float) -> None:
        """Fade the current sea out and ``rays`` in, from ``t`` over ``ramp_s``."""
        with self._lock:
            for layer in self._layers:
                if layer.fade_out_start >= NEVER:
                    layer.fade_out_start, layer.fade_out_duration = t, ramp_s
            if len(rays):
                self._layers.append(_Layer(rays, t, ramp_s))

    def prune(self, t: float) -> None:
        """Forget the layers faded out by ``t``, with a margin: several xdyn
        processes read the sea, and their times are not exactly in step."""
        with self._lock:
            self._layers = [l for l in self._layers if not l.faded_out(t - 5.0)]

    def components(self) -> Dict[str, List[float]]:
        """Every component with its fade, as xdyn's rays file and the renderer need them."""
        with self._lock:
            layers = list(self._layers)
        out = {k: [] for k in ("a", "psi", "omega", "k", "phase") + FADE_KEYS}
        for l in layers:
            n = len(l.rays)
            out["a"] += l.rays.a; out["psi"] += l.rays.psi; out["omega"] += l.rays.omega
            out["k"] += l.rays.k; out["phase"] += l.rays.phase
            out["fade_in_start"] += [l.fade_in_start] * n; out["fade_in_duration"] += [l.fade_in_duration] * n
            out["fade_out_start"] += [l.fade_out_start] * n; out["fade_out_duration"] += [l.fade_out_duration] * n
        return out
