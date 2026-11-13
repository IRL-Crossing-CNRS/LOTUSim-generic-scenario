"""Point xdyn's Ekman current at the scenario's weather.

xdyn computes the current (``- model: ekman current`` of each vehicle model,
EkmanUWCurrentModel in lxdyn), including the depth of the wind-driven layer
that a wind sets (``layer thickness follows wind``). This module only passes it
the weather, as waves.py passes the wave components:

- ``drive_with_weather`` writes into a vehicle model the 10 m wind U10, the
  direction it blows towards, the latitude of the world, the switch that lets
  xdyn derive the layer depth from the wind and, when the scenario gives them,
  the interior current and the seabed; it also points the model at the run's
  ``weather file``;
- ``write_weather`` writes that file: one change of wind, from one (U10,
  direction) to another between two simulation times. xdyn re-reads it when it
  changes and moves its current linearly from one wind to the other, over the
  same ramp as the waves and the wind.
"""

from __future__ import annotations

import os
import re
from typing import Dict, Tuple

#: (U10 m/s, direction the wind blows towards, deg from north towards east)
Wind = Tuple[float, float]

_CURRENT_ITEM = re.compile(r"^(\s*)- model:\s*ekman current\s*$")


def drive_with_weather(yaml_text: str, now: Wind, latitude_deg: float, weather_file: str,
                       interior: Dict | None = None) -> str:
    """Set the ``- model: ekman current`` of a vehicle model to the scenario's weather.

    ``interior``: optional ``{"speed_ms", "direction_deg", "seabed_m", "bottom_layer_m"}`` of
    the scenario, so that every vehicle model shares one ocean. Keys the model does not have
    are added at the end of its block; a model without an Ekman current is returned unchanged."""
    values = {
        "latitude": f"{{ value: {float(latitude_deg)!r}, unit: deg }}",
        "U10": f"{{ value: {float(now[0])!r}, unit: m/s }}",
        "wind orientation": f"{{ value: {float(now[1])!r}, unit: deg }}",
        "layer thickness follows wind": "true",
        "weather file": weather_file,
    }
    interior = interior or {}
    if "speed_ms" in interior:
        values["current velocity"] = f"{{ value: {float(interior['speed_ms'])!r}, unit: m/s }}"
        values["current orientation"] = f"{{ value: {float(interior.get('direction_deg', 0.0))!r}, unit: deg }}"
    if interior.get("seabed_m") is not None:
        values["seabed depth"] = repr(float(interior["seabed_m"]))
    if "bottom_layer_m" in interior:
        values["bottom layer thickness"] = f"{{ value: {float(interior['bottom_layer_m'])!r}, unit: m }}"

    lines = yaml_text.splitlines(keepends=True)
    out, i = [], 0
    while i < len(lines):
        m = _CURRENT_ITEM.match(lines[i].rstrip("\n"))
        out.append(lines[i])
        i += 1
        if not m:
            continue
        indent = m.group(1)
        key_indent = None
        seen = set()
        while i < len(lines):
            line = lines[i].rstrip("\n")
            stripped = line.strip()
            lead = len(line) - len(line.lstrip())
            if stripped and not stripped.startswith("#") and lead <= len(indent):
                break
            key = stripped.split(":", 1)[0] if ":" in stripped and not stripped.startswith("#") else None
            if key == "seabed file" and "seabed depth" in values:
                i += 1  # one seabed for the whole ocean: the scenario's depth replaces the model's map
                continue
            if key in values:
                key_indent = " " * lead
                lines[i] = f"{key_indent}{key}: {values[key]}\n"
                seen.add(key)
            elif key and key_indent is None:
                key_indent = " " * lead
            out.append(lines[i])
            i += 1
        pad = key_indent or (indent + "  ")
        blank = []
        while out and not out[-1].strip():  # keep the block's trailing blank lines after the new keys
            blank.insert(0, out.pop())
        out.extend(f"{pad}{k}: {v}\n" for k, v in values.items() if k not in seen)
        out.extend(blank)
    return "".join(out)


def write_weather(path: str, start_s: float, duration_s: float, old: Wind, new: Wind) -> None:
    """The change of wind xdyn's Ekman model reads (``weather file``), written next to ``path``
    and renamed over it, so xdyn never reads half a file."""
    tmp = f"{path}.{os.getpid()}.tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        f.write("start_s,duration_s,U10_from,wind_from_deg,U10_to,wind_to_deg\n")
        f.write(",".join(repr(float(v)) for v in (start_s, duration_s, *old, *new)) + "\n")
    os.replace(tmp, path)
