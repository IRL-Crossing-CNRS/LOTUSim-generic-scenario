"""The weather of a place and date, from the Copernicus Marine Service.

``"weather": {"date": "2024-08-10T12:00", "place": {"lat": 50.25, "lon": -4.25}}`` in a scenario
asks for the sea, the water column and the air measured there and then:

- waves: significant height (VHM0), peak period (VTPK) and mean direction (VMDR) of the global
  wave analysis and forecast (GLOBAL_ANALYSISFORECAST_WAV_001_027), 1/12 degree, every 3 hours;
- current: the eastward and northward velocity profiles (uo, vo) of the global ocean physics
  analysis and forecast (GLOBAL_ANALYSISFORECAST_PHY_001_024), 1/12 degree, every 6 hours,
  on its depth levels down to ``CURRENT_MAX_DEPTH_M``;
- wind: the 10 m wind (eastward_wind, northward_wind) of the global hourly sea-surface wind
  product (WIND_GLO_PHY_L4_NRT_012_004), 1/8 degree, every hour.

For each, the nearest time step and then the nearest sea point are used. The values are
downloaded once with the ``copernicusmarine`` command-line tool (``pip install
copernicusmarine``, then ``copernicusmarine login``) and cached as JSON, so later runs of the same
place and date work offline and read exactly the same weather. The cache directory is
``$LOTUSIM_WEATHER_CACHE``, by default ``~/.cache/lotusim/weather``.
"""

from __future__ import annotations

import csv
import json
import math
import os
import shutil
import subprocess
import tempfile
from datetime import datetime, timedelta
from pathlib import Path
from typing import Dict, List, Tuple

DATASET = "cmems_mod_glo_wav_anfc_0.083deg_PT3H-i"
CURRENT_DATASET = "cmems_mod_glo_phy-cur_anfc_0.083deg_PT6H-i"
WIND_DATASET = "cmems_obs-wind_glo_phy_nrt_l4_0.125deg_PT1H"
#: Half-width of the box searched for a sea point around the place (degrees).
BOX_DEG = 0.25
#: Deepest level of the current profile that is downloaded (m).
CURRENT_MAX_DEPTH_M = 200.0


def cache_dir() -> Path:
    return Path(os.environ.get("LOTUSIM_WEATHER_CACHE", Path.home() / ".cache" / "lotusim" / "weather"))


def _parse_date(date: str) -> datetime:
    return datetime.fromisoformat(str(date).replace("Z", ""))


def _cache_path(kind: str, lat: float, lon: float, when: datetime) -> Path:
    return cache_dir() / f"{kind}_{lat:.3f}_{lon:.3f}_{when:%Y-%m-%dT%H%M}.json"


def _subset(dataset: str, variables: List[str], lat: float, lon: float, when: datetime,
            hours: float, extra: List[str] = ()) -> List[Dict]:
    """Rows of a ``copernicusmarine subset`` CSV around (lat, lon) and ``when`` +- ``hours``."""
    exe = shutil.which("copernicusmarine")
    if exe is None:
        raise RuntimeError(
            "weather of a date: the 'copernicusmarine' tool is not installed "
            "(pip install copernicusmarine, then copernicusmarine login)"
        )
    # The tool is usually installed for the system Python, outside the simulation's environment:
    # run it with a clean Python path so that environment's packages do not shadow its own.
    env = {k: v for k, v in os.environ.items() if k not in ("PYTHONPATH", "PYTHONHOME", "PYTHONNOUSERSITE")}
    python = os.environ.get("LOTUSIM_COPERNICUS_PYTHON", "/usr/bin/python3")
    prefix = [python, exe] if os.path.exists(python) else [exe]
    with tempfile.TemporaryDirectory() as tmp:
        cmd = prefix + ["subset", "--dataset-id", dataset]
        for v in variables:
            cmd += ["--variable", v]
        cmd += [
            "--minimum-longitude", str(lon - BOX_DEG), "--maximum-longitude", str(lon + BOX_DEG),
            "--minimum-latitude", str(lat - BOX_DEG), "--maximum-latitude", str(lat + BOX_DEG),
            "--start-datetime", (when - timedelta(hours=hours)).isoformat(),
            "--end-datetime", (when + timedelta(hours=hours)).isoformat(),
            *extra,
            "--file-format", "csv", "--output-directory", tmp, "--output-filename", "out.csv",
        ]
        done = subprocess.run(cmd, capture_output=True, text=True, env=env)
        out = Path(tmp) / "out.csv"
        if done.returncode != 0 or not out.exists():
            raise RuntimeError(f"weather of a date: download of {dataset} failed: {done.stderr.strip()[-400:]}")
        rows = [r for r in csv.DictReader(out.open()) if all(r.get(v) not in (None, "", "nan") for v in variables)]
    if not rows:
        raise RuntimeError(f"weather of a date: no sea point of {dataset} within {BOX_DEG} deg of ({lat}, {lon})")
    return rows


def _nearest(rows: List[Dict], lat: float, lon: float, when: datetime) -> Dict:
    """The row nearest in time, then nearest in distance."""

    def cost(r: Dict) -> Tuple[float, float]:
        t = datetime.fromisoformat(r["time"])
        dist = math.hypot(float(r["latitude"]) - lat, (float(r["longitude"]) - lon) * math.cos(math.radians(lat)))
        return (abs((t - when).total_seconds()), dist)

    return min(rows, key=cost)


def _cached(kind: str, lat: float, lon: float, date: str, download) -> Dict:
    when = _parse_date(date)
    path = _cache_path(kind, lat, lon, when)
    if path.exists():
        return json.loads(path.read_text())
    value = download(lat, lon, when)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2))
    return value


def _download_waves(lat: float, lon: float, when: datetime) -> Dict:
    best = _nearest(_subset(DATASET, ["VHM0", "VTPK", "VMDR"], lat, lon, when, 3), lat, lon, when)
    came_from = float(best["VMDR"])
    return {
        "hs_m": float(best["VHM0"]),
        "tp_s": float(best["VTPK"]),
        # VMDR is the direction the waves come from; the scenario gives the one they travel to.
        "direction_deg": (came_from + 180.0) % 360.0,
        "source": DATASET,
        "time": best["time"],
        "lat": float(best["latitude"]),
        "lon": float(best["longitude"]),
    }


def _download_current(lat: float, lon: float, when: datetime) -> Dict:
    rows = _subset(CURRENT_DATASET, ["uo", "vo"], lat, lon, when, 6,
                   ["--minimum-depth", "0", "--maximum-depth", str(CURRENT_MAX_DEPTH_M)])
    # The point and time nearest to the request at the shallowest level, then that point's profile.
    top = min(float(r["depth"]) for r in rows)
    best = _nearest([r for r in rows if float(r["depth"]) == top], lat, lon, when)
    profile = sorted(
        (float(r["depth"]), float(r["vo"]), float(r["uo"]))
        for r in rows
        if r["time"] == best["time"] and r["latitude"] == best["latitude"] and r["longitude"] == best["longitude"]
    )
    return {
        # (depth m, north m/s, east m/s), the columns of a copernicus_current profile CSV.
        "profile": [list(p) for p in profile],
        "source": CURRENT_DATASET,
        "time": best["time"],
        "lat": float(best["latitude"]),
        "lon": float(best["longitude"]),
    }


def _download_wind(lat: float, lon: float, when: datetime) -> Dict:
    best = _nearest(_subset(WIND_DATASET, ["eastward_wind", "northward_wind"], lat, lon, when, 1), lat, lon, when)
    return {
        "east_ms": float(best["eastward_wind"]),
        "north_ms": float(best["northward_wind"]),
        "height_m": 10.0,
        "source": WIND_DATASET,
        "time": best["time"],
        "lat": float(best["latitude"]),
        "lon": float(best["longitude"]),
    }


def waves_of_date(lat: float, lon: float, date: str) -> Dict:
    """Hs (m), Tp (s) and travel direction (deg from north) at a place and date, cached."""
    return _cached("waves", lat, lon, date, _download_waves)


def current_of_date(lat: float, lon: float, date: str) -> Dict:
    """Current profile at a place and date, cached: ``profile`` is [[depth m, north m/s, east m/s], ...]."""
    return _cached("current", lat, lon, date, _download_current)


def wind_of_date(lat: float, lon: float, date: str) -> Dict:
    """10 m wind at a place and date (``east_ms``, ``north_ms``), cached."""
    return _cached("wind", lat, lon, date, _download_wind)


def write_current_profile(current: Dict, path: str) -> None:
    """The profile CSV the physics plugin replays (``copernicus_current``: depth_m, vx_north_ms, vy_east_ms)."""
    with open(path, "w", encoding="utf-8") as f:
        f.write("depth_m,vx_north_ms,vy_east_ms\n")
        for depth, north, east in current["profile"]:
            f.write(f"{depth!r},{north!r},{east!r}\n")


def weather_place_date(scenario: Dict):
    """``(lat, lon, date)`` of the scenario's ``"weather": {"date", "place"}`` entry, or None."""
    weather = scenario.get("weather") or {}
    if "date" not in weather:
        return None
    place = weather.get("place") or {}
    if "lat" not in place or "lon" not in place:
        raise ValueError('"weather": {"date": ...} needs "place": {"lat": ..., "lon": ...}')
    return float(place["lat"]), float(place["lon"]), str(weather["date"])
