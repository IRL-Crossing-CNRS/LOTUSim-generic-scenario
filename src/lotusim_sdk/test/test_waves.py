"""Tests of the shared sea: discretisation (lotusim_sdk.waves), the fading layers of a
live sea (lotusim_sdk.wave_layers) and the weather given to xdyn's Ekman current
(lotusim_sdk.ekman). xdyn computes the waves and the current itself.

The wave server reimplements xdyn's Airy model, so its sums are checked here
against the plain Python sum of WaveRays, which was itself checked against
xdyn's own output (equal to 3e-17 m on a rays file).
"""

import math

import numpy as np
import pytest

from lotusim_sdk.waves import (
    BEAUFORT, DEFAULTS, build_rays, cos2s, frequency_range, jonswap, replace_wave_model, sea_is_live,
    sea_is_shared, sea_state, strip_current_models, wave_number, wind_of_weather,
)
from lotusim_sdk import ekman


def rays_for(**kw):
    p = dict(DEFAULTS)
    p.update(kw)
    return build_rays(p)


def test_significant_height_is_exact():
    for hs, tp in ((0.5, 3.0), (2.0, 7.3), (4.0, 12.2)):
        r = rays_for(hs_m=hs, tp_s=tp, gamma=3.3)
        assert r.significant_height() == pytest.approx(hs, rel=1e-12)


def test_calm_sea_has_no_component():
    assert len(rays_for(hs_m=0.0)) == 0


def test_cos2s_integrates_to_one():
    n = 20000
    for s in (1.0, 2.0, 10.0):
        total = sum(cos2s(2 * math.pi * j / n, 0.3, s) for j in range(n)) * 2 * math.pi / n
        assert total == pytest.approx(1.0, rel=1e-6)


def test_jonswap_holds_the_energy_of_hs():
    hs, tp = 2.0, 7.0
    w = np.linspace(0.05, 10.0, 200000)
    m0 = np.trapezoid([jonswap(x, hs, tp, 3.3) for x in w], w)
    # Goda's normalisation of the peak enhancement is within a few percent.
    assert 4 * math.sqrt(m0) == pytest.approx(hs, rel=0.03)


def test_dispersion():
    assert wave_number(1.0, 0.0) == pytest.approx(1.0 / 9.81)
    k = wave_number(1.0, 5.0)
    assert 1.0 == pytest.approx(math.sqrt(9.81 * k * math.tanh(k * 5.0)))


def test_frequencies_resolve_the_peak_and_do_not_repeat():
    r = rays_for(hs_m=2.0, tp_s=7.3, gamma=3.3)
    omegas = sorted(set(r.omega))
    wp = 2 * math.pi / 7.3
    w0, w1 = frequency_range(dict(DEFAULTS, tp_s=7.3))
    assert (w0, w1) == pytest.approx((0.5 * wp, 3.0 * wp))
    # Bin width about the JONSWAP peak width for gamma 3.3 (sigma = 0.07 wp).
    assert (w1 - w0) / DEFAULTS["nfreq"] < 0.08 * wp
    # Random frequencies within the bins: no common spacing, so no period.
    gaps = np.diff(omegas)
    assert np.std(gaps) > 0.1 * np.mean(gaps)


def test_seed_fixes_the_sea():
    a, b = rays_for(seed=7), rays_for(seed=7)
    assert a.omega == b.omega and a.phase == b.phase
    assert rays_for(seed=8).phase != a.phase


def test_beaufort_sea_state():
    for b, (wind, hs) in BEAUFORT.items():
        p = sea_state({"weather": {"beaufort": b}})
        assert p["hs_m"] == hs
        if hs > 0:
            assert p["tp_s"] == pytest.approx(4.4 * math.sqrt(hs))
            assert math.sqrt(13 * hs) <= p["tp_s"] <= 5 * math.sqrt(hs)
        assert p["gamma"] == 3.3


def test_live_and_shared_flags():
    assert not sea_is_live({"weather": {"beaufort": 3}})
    assert sea_is_live({"weather": {"beaufort": 3, "follow_wind": True}})
    assert sea_is_live({"waves": {"hs_m": 1.0, "live": True}})
    assert sea_is_shared({"waves": {"hs_m": 1.0}})
    assert not sea_is_shared({"waves": {"hs_m": 1.0, "shared": False}})


YAML = """environment models:
  - model: waves
    discretization:
      nfreq: 16
    spectra:
      - model: airy
        seed of the random data generator: 2
  - model: ekman current
    wind speed: {value: 5, unit: m/s}
    layers: 3
  - model: no wind
bodies:
  - name: x
"""


def test_replace_wave_model_variants():
    rays_yaml = replace_wave_model(YAML, "/tmp/rays.csv")
    assert "rays from file: /tmp/rays.csv" in rays_yaml and "nfreq: 16" not in rays_yaml
    live_yaml = replace_wave_model(YAML, "/tmp/rays.csv", reload=True)
    assert "reload rays file: true" in live_yaml
    own = replace_wave_model(YAML, None, own_sea=(dict(DEFAULTS, hs_m=1.0, tp_s=5.0, gamma=3.3), 9))
    assert "seed of the random data generator: 9" in own and "Hs: {value: 1.0, unit: m}" in own
    for text in (rays_yaml, live_yaml, own):
        assert "- model: ekman current" in text and "- model: no wind" in text


def test_strip_current_models():
    text = strip_current_models(YAML)
    assert "ekman current" not in text and "layers: 3" not in text
    assert "- model: waves" in text and "- model: no wind" in text and "bodies:" in text


EKMAN_YAML = """environment models:
  - model: ekman current
    seabed file: seabed/seabed.png
    latitude: { value: 47, unit: deg }
    top layer thickness: { value: 10, unit: m }
    bottom layer thickness: { value: 20, unit: m }
    current velocity: { value: 0.35, unit: m/s }
    current orientation: { value: 30, unit: deg }
    U10: { value: 1, unit: m/s }
    wind orientation: { value: 20, unit: deg }
  - model: no wind
bodies:
  - name: x
    latitude: unchanged
"""


def test_weather_drives_xdyn_ekman_model():
    text = ekman.drive_with_weather(EKMAN_YAML, (9.35, 40.0), 50.33, "/tmp/w.csv")
    assert "U10: { value: 9.35, unit: m/s }" in text
    assert "wind orientation: { value: 40.0, unit: deg }" in text
    assert "latitude: { value: 50.33, unit: deg }" in text
    assert "layer thickness follows wind: true" in text  # xdyn derives the layer depth from the wind
    assert "    weather file: /tmp/w.csv\n  - model: no wind" in text  # added inside the block
    assert "current velocity: { value: 0.35, unit: m/s }" in text  # the model's interior is kept
    assert "latitude: unchanged" in text  # nothing outside the block moves
    # The scenario's interior current and seabed give every model the same ocean.
    one = ekman.drive_with_weather(EKMAN_YAML, (9.35, 40.0), 50.33, "/tmp/w.csv",
                                   {"speed_ms": 0.1, "direction_deg": 90.0, "seabed_m": 60.0})
    assert "current velocity: { value: 0.1, unit: m/s }" in one
    assert "current orientation: { value: 90.0, unit: deg }" in one
    assert "seabed depth: 60.0" in one and "seabed file" not in one
    assert ekman.drive_with_weather("environment models:\n  - model: no waves\n", (9.35, 40.0), 50.33, "w") == \
        "environment models:\n  - model: no waves\n"


def test_weather_file_holds_one_change_of_wind(tmp_path):
    path = str(tmp_path / "ekman_weather.csv")
    ekman.write_weather(path, 80.0, 30.0, (4.4, 0.0), (9.35, 0.0))
    header, row = open(path).read().splitlines()
    assert header.split(",")[0] == "start_s" and len(row.split(",")) == 6
    assert [float(v) for v in row.split(",")] == [80.0, 30.0, 4.4, 0.0, 9.35, 0.0]


def test_wind_of_a_force_blows_with_the_waves():
    east, north = wind_of_weather(sea_state({"weather": {"beaufort": 5, "direction_deg": 90.0}}))
    assert east == pytest.approx(BEAUFORT[5][0]) and north == pytest.approx(0.0, abs=1e-12)
    assert wind_of_weather({"hs_m": 1.0}) is None


# --- the wave server ---------------------------------------------------------

def test_live_sea_layers_carry_their_fades():
    from lotusim_sdk.wave_layers import NEVER, WaveField
    a, b = rays_for(hs_m=1.0, tp_s=5.0), rays_for(hs_m=2.0, tp_s=7.0, seed=9)
    field = WaveField(a)
    field.change(b, 80.0, 30.0)
    c = field.components()
    n = len(a)
    assert len(c["a"]) == n + len(b)
    assert c["fade_out_start"][0] == 80.0 and c["fade_out_duration"][0] == 30.0
    assert c["fade_in_start"][n] == 80.0 and c["fade_out_start"][n] == NEVER
    field.prune(80.0 + 30.0 + 6.0)  # old sea gone 5 s after its fade ended
    assert len(field.components()["a"]) == len(b)
