"""One sea state shared by xdyn and the renderer.

xdyn draws its own random phases when a vehicle YAML declares a wave
``spectra`` block, and every vehicle YAML has its own block, so neither the
other vehicles nor Unity can know the surface it computes. This module builds
the wave components once, from the scenario's ``"waves"`` block, and hands
the very same list to both sides:

- xdyn reads it through its ``spectra from a list of rays`` input (a CSV of
  ``a, psi, omega, k, phase``), written into every xdyn vehicle YAML, and
  evaluates it with its own Airy model. When the sea state can change during
  the run (``"live": true``, ``"follow_wind": true`` or a ``"schedule"``), the
  wave server node rewrites that CSV at each change, with four fade columns
  per component, and xdyn re-reads it (``reload rays file: true``); every
  component is weighted by sqrt(u_in (1 - u_out)) (lotusim_sdk.wave_layers);
- the renderer receives it on a ROS topic and evaluates the same sum, with
  the same fades.

Directions: the circle is cut into ``ndir`` equal sectors and each component
takes a seeded random direction inside its sector, so that the crests of all
frequencies do not line up into a lattice.

Frequencies: one per bin of equal width over [omega_min, omega_max], drawn
at random inside its bin (seeded). Evenly spaced frequencies would make the
sea repeat itself every 2 pi / d_omega (about 20 s with 16 frequencies over
0.1-4 rad/s); random ones within the bins do not. Without an explicit range,
the bins cover [0.5, 3] times the peak frequency, which holds about 99 % of a
JONSWAP sea's energy and resolves its peak.

Surface elevation, in xdyn's NED frame (z down), as in xdyn's ``Airy`` model::

    zeta(x, y, t) = - sum_i a_i * sin(k_i * (x cos psi_i + y sin psi_i) - omega_i * t + phase_i)

``psi`` is the direction the component travels to, from north (x) towards
east (y). ``t`` is the simulation time the physics plugin sends to xdyn.
"""

from __future__ import annotations

import math
import os
import re
from dataclasses import dataclass, field
from typing import Dict, List

G = 9.81

#: Scenario "waves" block keys and their defaults.
DEFAULTS = {
    "hs_m": 0.5,
    "tp_s": 3.0,
    "gamma": 1.2,
    "direction_deg": 0.0,
    "spreading_s": 2.0,
    "nfreq": 32,
    "ndir": 8,
    # None: [0.5, 3] times the peak frequency 2 pi / tp_s.
    "omega_min": None,
    "omega_max": None,
    "frequency_jitter": True,
    "energy_fraction": 0.99,
    "seed": 2,
    "depth_m": 0.0,
}


#: Beaufort force -> (10 m wind U10, m/s; probable significant wave height in
#: the open sea, m). U10: middle of the WMO class, the same values as the Ekman
#: current model's Beaufort table (35 m/s for force 12). Hs: WMO "probable wave
#: height" (WMO-No. 558, Manual on Marine Meteorological Services). One table, so
#: that a force gives the drones, the Ekman layer and the sea the same weather.
BEAUFORT = {
    0: (0.1, 0.0), 1: (0.9, 0.1), 2: (2.45, 0.2), 3: (4.4, 0.6), 4: (6.7, 1.0),
    5: (9.35, 2.0), 6: (12.3, 3.0), 7: (15.5, 4.0), 8: (18.95, 5.5), 9: (22.6, 7.0),
    10: (26.45, 9.0), 11: (30.55, 11.5), 12: (35.0, 14.0),
}


def sea_state(scenario: Dict) -> Dict:
    """The wave parameters a scenario asks for, or {} for none.

    ``"waves"`` gives them explicitly. ``"weather": {"date": ..., "place": {"lat", "lon"}}`` reads
    Hs, Tp and the mean direction measured there and then (lotusim_sdk.weather_of_date), with a
    JONSWAP peak factor of 3.3. ``"weather": {"beaufort": B}`` derives
    Hs from the WMO table above and Tp = 4.4 sqrt(Hs), inside the JONSWAP range
    3.6 sqrt(Hs) <= Tp <= 5 sqrt(Hs) of DNV-RP-C205, with a JONSWAP peak factor
    of 3.3. The WMO heights are those of a growing sea: the period of a fully
    developed one, under the same wind, would be up to twice as long. The frequency range
    follows the peak ([0.5, 3] omega_p, see frequency_range). Any key of ``"waves"`` or of ``"weather"`` that
    matches DEFAULTS overrides the derived value.
    """
    weather = scenario.get("weather") or {}
    waves = scenario.get("waves") or {}
    if not weather and not waves:
        return {}
    p = {}
    if "beaufort" in weather:
        b = int(round(float(weather["beaufort"])))
        wind, hs = BEAUFORT[max(0, min(12, b))]
        # Force 0 (Hs 0) keeps the Pierson-Moskowitz period of its wind: no waves, but a
        # valid frequency range.
        tp = 4.4 * math.sqrt(hs) if hs > 0 else 2.0 * math.pi * 1.075 * wind / (0.877 * G)
        p.update(hs_m=hs, tp_s=tp, gamma=3.3, wind_ms=wind, beaufort=max(0, min(12, b)))
    if "date" in weather:
        # The waves measured at a place and date (Copernicus Marine wave analysis).
        from lotusim_sdk.weather_of_date import waves_of_date

        place = weather.get("place") or {}
        if "lat" not in place or "lon" not in place:
            raise ValueError('"weather": {"date": ...} needs "place": {"lat": ..., "lon": ...}')
        sea = waves_of_date(float(place["lat"]), float(place["lon"]), str(weather["date"]))
        p.update(hs_m=sea["hs_m"], tp_s=sea["tp_s"], direction_deg=sea["direction_deg"], gamma=3.3)
    p.update({k: v for k, v in weather.items() if k in DEFAULTS})
    p.update({k: v for k, v in waves.items() if k in DEFAULTS})
    return p


def sea_is_live(scenario: Dict) -> bool:
    """Whether the sea state may change during the run: ``"live": true`` in
    ``"waves"`` or ``"weather"``, ``"weather": {"follow_wind": true}``, or a
    ``"weather": {"schedule": [...]}`` of changes at given times.
    Either way xdyn evaluates the sea with its own Airy model, from a rays
    file; a live one is rewritten by the wave server at each change, with the
    fades of the change, and xdyn re-reads it (``reload rays file: true``)."""
    weather = scenario.get("weather") or {}
    waves = scenario.get("waves") or {}
    return bool(weather.get("follow_wind") or weather.get("live") or waves.get("live") or weather.get("schedule"))


def sea_is_shared(scenario: Dict) -> bool:
    """False for ``"shared": false``: every vehicle model draws its own random
    sea of the same spectrum and the renderer draws its own, as before the
    shared sea existed. Kept as the baseline of the shared sea."""
    weather = scenario.get("weather") or {}
    waves = scenario.get("waves") or {}
    return bool(waves.get("shared", weather.get("shared", True)))


def frequency_range(p: Dict) -> tuple:
    """[omega_min, omega_max] of the discretisation (rad/s)."""
    wp = 2.0 * math.pi / float(p["tp_s"])
    w0 = p.get("omega_min")
    w1 = p.get("omega_max")
    return (0.5 * wp if w0 is None else float(w0)), (3.0 * wp if w1 is None else float(w1))


@dataclass
class WaveRays:
    """Flat list of wave components, one entry per (frequency, direction)."""

    a: List[float] = field(default_factory=list)
    psi: List[float] = field(default_factory=list)
    omega: List[float] = field(default_factory=list)
    k: List[float] = field(default_factory=list)
    phase: List[float] = field(default_factory=list)
    depth_m: float = 0.0

    def __len__(self) -> int:
        return len(self.a)

    def significant_height(self) -> float:
        """Hs = 4 sqrt(m0), m0 = sum(a^2) / 2."""
        return 4.0 * math.sqrt(sum(a * a for a in self.a) / 2.0)



def jonswap(omega: float, hs: float, tp: float, gamma: float) -> float:
    """JONSWAP spectral density S(omega), m^2.s/rad."""
    wp = 2.0 * math.pi / tp
    sigma = 0.07 if omega <= wp else 0.09
    r = math.exp(-((omega - wp) ** 2) / (2.0 * sigma * sigma * wp * wp))
    pm = 5.0 / 16.0 * hs * hs * wp**4 / omega**5 * math.exp(-1.25 * (wp / omega) ** 4)
    return pm * (1.0 - 0.287 * math.log(gamma)) * gamma**r


def cos2s(psi: float, psi0: float, s: float) -> float:
    """cos-2s directional spreading, normalised to 1 over [0, 2 pi)."""
    norm = math.gamma(s + 1.0) / (2.0 * math.sqrt(math.pi) * math.gamma(s + 0.5))
    return norm * abs(math.cos((psi - psi0) / 2.0)) ** (2.0 * s)


def wave_number(omega: float, depth: float) -> float:
    """Linear dispersion omega^2 = g k tanh(k h); depth 0 means infinite."""
    k = omega * omega / G
    if depth <= 0.0:
        return k
    for _ in range(50):
        f = G * k * math.tanh(k * depth) - omega * omega
        df = G * math.tanh(k * depth) + G * k * depth / math.cosh(k * depth) ** 2
        k_next = k - f / df
        if abs(k_next - k) < 1e-12:
            return k_next
        k = k_next
    return k


def build_rays(params: Dict) -> WaveRays:
    """Discretise a JONSWAP x cos-2s sea state into rays with seeded phases.

    Frequencies: one per bin of equal width over [omega_min, omega_max]
    (default [0.5, 3] omega_p), at a seeded random place inside the bin;
    directions evenly spaced over the full circle, each component at its own
    seeded place inside its direction sector. Components are kept largest first
    until ``energy_fraction`` of the energy is reached, as xdyn does, then
    the amplitudes are scaled so that the kept set has exactly the requested
    Hs.
    """
    import random

    p = dict(DEFAULTS)
    p.update(params or {})
    if float(p["hs_m"]) <= 0.0:
        return WaveRays(depth_m=float(p["depth_m"]))  # calm sea: no component
    nfreq, ndir = int(p["nfreq"]), int(p["ndir"])
    w0, w1 = frequency_range(p)
    rng = random.Random(int(p["seed"]))
    # One frequency per bin of width dw, at random inside the bin (or at its
    # centre without jitter), so that no common spacing makes the sea periodic.
    dw = (w1 - w0) / nfreq
    jitter = bool(p.get("frequency_jitter", True))
    omegas = [w0 + (i + (rng.random() if jitter else 0.5)) * dw for i in range(nfreq)]
    psis = [2.0 * math.pi * j / ndir for j in range(ndir)]
    psi0 = math.radians(float(p["direction_deg"]))
    dpsi = 2.0 * math.pi / ndir if ndir > 1 else 1.0
    depth = float(p["depth_m"])

    # Sharing the same 8 directions, all frequencies lined their crests up into a
    # visible lattice in a storm. Its own generator keeps the phases of a seed unchanged.
    dir_rng = random.Random(int(p["seed"]) + 1)
    comps = []
    for w in omegas:
        s_w = jonswap(w, float(p["hs_m"]), float(p["tp_s"]), float(p["gamma"]))
        k = wave_number(w, depth)
        for psi_c in psis:
            psi = psi_c + ((dir_rng.random() - 0.5) * dpsi if jitter else 0.0)
            a = math.sqrt(2.0 * s_w * cos2s(psi, psi0, float(p["spreading_s"])) * dw * dpsi)
            comps.append((a, psi, w, k, rng.uniform(0.0, 2.0 * math.pi)))

    total = sum(c[0] ** 2 for c in comps)
    rays = WaveRays(depth_m=depth)
    kept = 0.0
    for c in sorted(comps, key=lambda c: c[0], reverse=True):
        if kept >= float(p["energy_fraction"]) * total:
            break
        kept += c[0] ** 2
        rays.a.append(c[0])
        rays.psi.append(c[1])
        rays.omega.append(c[2])
        rays.k.append(c[3])
        rays.phase.append(c[4])

    # Truncating the frequency range and dropping the weakest components
    # lowers the energy a little: scale back to the requested Hs.
    scale = float(p["hs_m"]) / rays.significant_height()
    rays.a = [a * scale for a in rays.a]
    return rays


def write_rays_csv(rays: WaveRays, path: str) -> None:
    """The CSV xdyn reads from ``rays from file``."""
    with open(path, "w", encoding="utf-8") as f:
        f.write("a,psi,omega,k,phase\n")
        for row in zip(rays.a, rays.psi, rays.omega, rays.k, rays.phase):
            f.write(",".join(repr(v) for v in row) + "\n")


FADE_COLUMNS = ("fade_in_start", "fade_in_duration", "fade_out_start", "fade_out_duration")


def write_components_csv(components: Dict, path: str) -> None:
    """The rays of a live sea with the fade of each one (``WaveField.components``),
    for xdyn's ``reload rays file``. Written next to ``path`` and renamed over
    it, so a reader never sees half a file."""
    tmp = f"{path}.{os.getpid()}.tmp"
    columns = ("a", "psi", "omega", "k", "phase") + FADE_COLUMNS
    with open(tmp, "w", encoding="utf-8") as f:
        f.write(",".join(columns) + "\n")
        for row in zip(*(components[c] for c in columns)):
            f.write(",".join(repr(float(v)) for v in row) + "\n")
    os.replace(tmp, path)


def _waves_block(csv_path: str, depth: float, indent: str, reload: bool = False) -> str:
    i1, i2, i3 = indent + "  ", indent + "    ", indent + "      "
    return (
        f"{indent}- model: waves\n"
        f"{i1}# Written by the scenario launcher from the scenario's \"waves\" block:\n"
        f"{i1}# the same components as every other vehicle and the renderer.\n"
        f"{i1}spectra from a list of rays:\n"
        f"{i2}- model: airy\n"
        f"{i3}rays from file: {csv_path}\n"
        + (f"{i3}# Rewritten by the wave server at each change of the sea state.\n"
           f"{i3}reload rays file: true\n" if reload else "")
        + f"{i3}depth: {{value: {depth}, unit: m}}\n"
        f"{i3}stretching:\n"
        f"{i3}  delta: 1\n"
        f"{i3}  h: {{unit: m, value: 0}}\n"
    )


def _no_waves_block(indent: str) -> str:
    return (
        f"{indent}- model: no waves\n"
        f"{indent}  # Calm sea, from the scenario's \"waves\"/\"weather\" block.\n"
        f"{indent}  constant sea elevation in NED frame: {{value: 0, unit: m}}\n"
    )


def _own_sea_block(params: Dict, seed: int, indent: str) -> str:
    """xdyn's own spectral wave model with the scenario's sea state: xdyn draws
    its own phases from ``seed``, so no other vehicle and no renderer knows the
    surface (the ``"shared": false`` baseline)."""
    w0, w1 = frequency_range(params)
    i1, i2, i3, i4 = indent + "  ", indent + "    ", indent + "      ", indent + "        "
    depth = float(params["depth_m"])
    return (
        f"{indent}- model: waves\n"
        f"{i1}# Written by the scenario launcher: \"shared\": false, this vehicle's own sea.\n"
        f"{i1}discretization:\n"
        f"{i2}nfreq: {int(params['nfreq'])}\n"
        f"{i2}ndir: {int(params['ndir'])}\n"
        f"{i2}omega min: {{value: {w0!r}, unit: rad/s}}\n"
        f"{i2}omega max: {{value: {w1!r}, unit: rad/s}}\n"
        f"{i2}energy fraction: {float(params['energy_fraction'])!r}\n"
        f"{i2}equal energy bins: false\n"
        f"{i1}spectra:\n"
        f"{i2}- model: airy\n"
        f"{i3}depth: {{value: {depth if depth > 0 else 1000.0}, unit: m}}\n"
        f"{i3}seed of the random data generator: {int(seed)}\n"
        f"{i3}stretching:\n"
        f"{i3}  delta: 1\n"
        f"{i3}  h: {{unit: m, value: 0}}\n"
        f"{i3}directional spreading:\n"
        f"{i4}type: cos2s\n"
        f"{i4}s: {float(params['spreading_s'])!r}\n"
        f"{i4}waves propagating to: {{value: {float(params['direction_deg'])!r}, unit: deg}}\n"
        f"{i3}spectral density:\n"
        f"{i4}type: jonswap\n"
        f"{i4}Hs: {{value: {float(params['hs_m'])!r}, unit: m}}\n"
        f"{i4}Tp: {{value: {float(params['tp_s'])!r}, unit: s}}\n"
        f"{i4}gamma: {float(params['gamma'])!r}\n"
    )


_CURRENT_ITEM = re.compile(r"^(\s*)- model:\s*ekman current\s*$")


def strip_current_models(yaml_text: str) -> str:
    """Remove the vehicle YAML's own current model (``- model: ekman
    current``), for a run whose current is the measured profile of a date
    replayed by the physics plugin: keeping both would apply the current
    twice."""
    lines = yaml_text.splitlines(keepends=True)
    out, i = [], 0
    while i < len(lines):
        m = _CURRENT_ITEM.match(lines[i].rstrip("\n"))
        if m:
            indent = m.group(1)
            i += 1
            while i < len(lines):
                stripped = lines[i].strip()
                lead = len(lines[i]) - len(lines[i].lstrip())
                if stripped and not stripped.startswith("#") and lead <= len(indent):
                    break
                i += 1
            continue
        out.append(lines[i])
        i += 1
    return "".join(out)


def wind_of_weather(params: Dict) -> tuple | None:
    """(east, north) 10 m wind, m/s, of a Beaufort weather: U10 of the force,
    blowing towards the waves' direction. None when the weather has no force."""
    if "beaufort" not in params:
        return None
    u10 = BEAUFORT[int(params["beaufort"])][0]
    psi = math.radians(float(params.get("direction_deg", 0.0)))
    return u10 * math.sin(psi), u10 * math.cos(psi)


_WAVE_ITEM = re.compile(r"^(\s*)- model:\s*(no waves|waves|grpc)\s*$")  # grpc: models of older runs


def replace_wave_model(yaml_text: str, csv_path: str | None, depth: float = 0.0,
                       own_sea: tuple | None = None, reload: bool = False) -> str:
    """Swap a vehicle YAML's wave environment model for the shared rays in
    ``csv_path``, else for a calm sea. ``own_sea=(params, seed)`` writes instead xdyn's own spectral model
    of that sea state, with its own phases (the unshared baseline).

    Line-based, so the comments elsewhere in the file survive. The item runs
    from its ``- model:`` line to the next line at the same or a lower
    indentation. A YAML with no wave item (some declare none) gets one
    inserted at the top of ``environment models:``.
    """
    def block(indent: str) -> str:
        if own_sea is not None:
            return _own_sea_block(own_sea[0], own_sea[1], indent)
        return _waves_block(csv_path, depth, indent, reload) if csv_path else _no_waves_block(indent)

    lines = yaml_text.splitlines(keepends=True)
    out, i, replaced = [], 0, False
    while i < len(lines):
        m = _WAVE_ITEM.match(lines[i].rstrip("\n"))
        if m and not replaced:
            indent = m.group(1)
            i += 1
            while i < len(lines):
                stripped = lines[i].strip()
                lead = len(lines[i]) - len(lines[i].lstrip())
                if stripped and not stripped.startswith("#") and lead <= len(indent):
                    break
                i += 1
            out.append(block(indent))
            replaced = True
            continue
        out.append(lines[i])
        i += 1
    if replaced:
        return "".join(out)

    for j, line in enumerate(out):
        if line.startswith("environment models:"):
            nxt = next((l for l in out[j + 1 :] if l.strip().startswith("- ")), "  - ")
            indent = nxt[: len(nxt) - len(nxt.lstrip())]
            out.insert(j + 1, block(indent))
            return "".join(out)
    raise ValueError("no 'environment models:' section found")
