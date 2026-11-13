"""Prepare the weather of a mission for xdyn and the wave server, before xdyn starts.

Run by scenario_launch.sh: ``python3 -m simulation_run.prepare_weather --config <scenario.json>
--out-dir <run>/waves --world-file <world.sdf> <xdyn model YAML>...``

Gives every xdyn vehicle model the scenario's sea state, and its current.

Reads the scenario JSON's "waves" or "weather" block (see
lotusim_sdk.waves.sea_state), builds the wave components once
(lotusim_sdk.waves), writes them to <out-dir>/wave_rays.csv and writes, for
each xdyn model given, a copy in <out-dir> whose wave environment model is:

- the rays of that CSV, evaluated by xdyn's own Airy model, when the sea
  state is fixed for the run (the default);
- the same CSV, with the fade of each component and ``reload rays file``,
  when the sea state can change during the run ("live": true, or "weather":
  {"follow_wind": true}): the wave server rewrites it at each change and xdyn
  re-reads it, so the physics stays as fast as with a fixed sea;
- xdyn's own random sea of the same spectrum, one per vehicle model, with
  "shared": false (the baseline before the shared sea).

The launcher then starts xdyn on the copies. The CSV keeps a record of the
initial sea.

With "weather": {"date", "place"} (and unless "currents": false), the current
profile of that place and date is also downloaded (lotusim_sdk.weather_of_date)
and written to <out-dir>/current_profile.csv, and the models' own current
(``- model: ekman current``) is removed from the copies, since the physics
plugin replays the profile instead (see agents_manager).

With "weather": {"beaufort": B} (and unless "currents": false), xdyn's Ekman
current of every model is driven by that weather (lotusim_sdk.ekman): U10 of
force B blowing towards the waves' direction, at the latitude of the world
(``--world-file``), xdyn deriving the layer depth from that wind, and, when the scenario
gives them ("weather": {"current": {"speed_ms", "direction_deg"}, "seabed_m",
"bottom_layer_m"}), one interior current and seabed for every model. The models
read <out-dir>/ekman_weather.csv, which the wave server rewrites when the force
changes during the run, so xdyn moves the current with the sea and the wind.
"""

import argparse
import json
import os
import re
import sys
import xml.etree.ElementTree as ET

from lotusim_sdk.waves import (
    BEAUFORT, DEFAULTS, build_rays, replace_wave_model, sea_is_live, sea_is_shared, sea_state,
    strip_current_models, write_components_csv, write_rays_csv,
)
from lotusim_sdk import ekman
from lotusim_sdk.wave_layers import WaveField


def world_latitude(world_file):
    """Latitude (deg) of the world's live <spherical_coordinates>, or None.
    Comments are stripped first: energy.world keeps an older block commented out."""
    if not world_file or not os.path.isfile(world_file):
        return None
    with open(world_file, encoding="utf-8") as f:
        text = re.sub(r"<!--.*?-->", "", f.read(), flags=re.S)
    try:
        node = ET.fromstring(text).find(".//spherical_coordinates/latitude_deg")
    except ET.ParseError:
        return None
    return float(node.text) if node is not None and node.text else None


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True)
    parser.add_argument("--out-dir", required=True)
    parser.add_argument("--world-file", default=None, help="world SDF, for the latitude of the Ekman layer")
    parser.add_argument("models", nargs="*")
    args = parser.parse_args()

    with open(args.config, encoding="utf-8") as f:
        scenario = json.load(f)
    params = dict(DEFAULTS)
    params.update(sea_state(scenario))
    rays = build_rays(params)
    live, shared = sea_is_live(scenario), sea_is_shared(scenario)

    os.makedirs(args.out_dir, exist_ok=True)
    csv_path = None
    if live and shared:
        # Written even for a calm start: the wave server fills it when the sea rises.
        csv_path = os.path.abspath(os.path.join(args.out_dir, "wave_rays.csv"))
        write_components_csv(WaveField(rays).components(), csv_path)
    elif len(rays):
        csv_path = os.path.abspath(os.path.join(args.out_dir, "wave_rays.csv"))
        write_rays_csv(rays, csv_path)
    mode = ("own sea per vehicle model (not shared)" if not shared
            else "rays evaluated by xdyn, re-read at each change" if live else "rays evaluated by xdyn")
    beaufort = f"Beaufort {params['beaufort']}, " if "beaufort" in params else ""
    print(
        f"[prepare_weather] {beaufort}{len(rays)} components, Hs {params['hs_m']} m, "
        f"Tp {params['tp_s']:.2f} s, towards {params['direction_deg']} deg, {mode}"
        + (f" -> {csv_path}" if csv_path else " (calm sea)"),
        file=sys.stderr,
    )

    profile = None
    weather = scenario.get("weather") or {}
    if "date" in weather and weather.get("currents", True):
        from lotusim_sdk.weather_of_date import current_of_date, weather_place_date, write_current_profile

        lat, lon, date = weather_place_date(scenario)
        current = current_of_date(lat, lon, date)
        profile = os.path.abspath(os.path.join(args.out_dir, "current_profile.csv"))
        write_current_profile(current, profile)
        north, east = current["profile"][0][1], current["profile"][0][2]
        print(
            f"[prepare_weather] current of {date} at ({current['lat']}, {current['lon']}), {current['time']}: "
            f"{len(current['profile'])} levels, surface {north:.3f} m/s north, {east:.3f} m/s east -> {profile}",
            file=sys.stderr,
        )

    driven = None
    if "beaufort" in params and not profile and weather.get("currents", True):
        latitude = world_latitude(args.world_file)
        if latitude is None:
            latitude = 45.0
            print("[prepare_weather] the world has no latitude: Ekman current at 45 deg N", file=sys.stderr)
        now = (BEAUFORT[int(params["beaufort"])][0], float(params["direction_deg"]))
        weather_file = os.path.abspath(os.path.join(args.out_dir, "ekman_weather.csv"))
        ekman.write_weather(weather_file, 0.0, 0.0, now, now)
        interior = dict(weather.get("current") or {})
        interior.update({k: weather[k] for k in ("seabed_m", "bottom_layer_m") if k in weather})
        driven = (now, latitude, weather_file, interior)
        print(
            f"[prepare_weather] Ekman current of force {params['beaufort']}: U10 {now[0]} m/s towards "
            f"{now[1]:.0f} deg, latitude {latitude:.4f} deg -> {weather_file}",
            file=sys.stderr,
        )

    for n, model in enumerate(args.models):
        if not os.path.isfile(model):
            continue
        with open(model, encoding="utf-8") as f:
            text = f.read()
        if not shared and len(rays):
            # Each vehicle model its own phases, as when every YAML had its own sea.
            text = replace_wave_model(text, None, own_sea=(params, int(params["seed"]) + 1 + n))
        else:
            text = replace_wave_model(text, csv_path, float(params["depth_m"]), reload=live)
        if profile:
            text = strip_current_models(text)
        elif driven:
            text = ekman.drive_with_weather(text, driven[0], driven[1], driven[2], driven[3])
        out = os.path.join(args.out_dir, os.path.basename(model))
        with open(out, "w", encoding="utf-8") as f:
            f.write(text)


if __name__ == "__main__":
    main()
