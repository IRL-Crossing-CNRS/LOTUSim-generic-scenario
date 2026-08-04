"""Regression tests for the Larsen wake model.

The point of this model is that it reproduces CFD-validated numbers, so the
published benchmark case is locked down here: if a refactor changes it, the
model is no longer the one the paper validated. Pure numpy — no ROS needed.
"""

import numpy as np
import pytest

from lotusim_sdk.agents.environment.wake.larsen import LarsenWakeModel

# NREL 5MW reference turbine, the machine every upstream protocol was run on.
NREL_5MW = dict(diameter=126.0, ct=0.75, cp=0.498, air_density=1.225,
                ambient_ti=0.08, cut_in=3.0, cut_out=25.0)
HUB = 90.0

# Validation layout A: 3 turbines in line, 7D spacing, wind straight down the
# row. ENU: (x, y, z) with y running downwind and z the hub height.
LAYOUT_A = [(0.0, 0.0, HUB), (0.0, 882.0, HUB), (0.0, 1764.0, HUB)]


@pytest.fixture
def model():
    return LarsenWakeModel(**NREL_5MW)


def test_reproduces_published_benchmark(model):
    """P2-1, layout A at 10 m/s — the CFD-validated reference result."""
    _, velocities, _ = model.wind_speeds_full(LAYOUT_A, [0.0, 10.0])
    assert velocities == [10.0, 7.71, 6.13]

    powers_mw = [round(model.power(v, hub_height=HUB) / 1e6, 3) for v in velocities]
    assert powers_mw == [3.724, 1.707, 0.858]


def test_deficit_deepens_down_the_row(model):
    """Each successive rotor sees less wind than the one ahead of it.

    Guards the sequential local superposition: an RSS combination of
    freestream deficits saturates instead, which is what made the older models
    over-predict deep rows.
    """
    _, velocities, _ = model.wind_speeds_full(LAYOUT_A, [0.0, 10.0])
    assert velocities[0] > velocities[1] > velocities[2]


def test_sorted_upstream_to_downstream(model):
    """Output order follows the wind, whatever order the layout came in."""
    reversed_layout = list(reversed(LAYOUT_A))
    turbines_sorted, velocities, _ = model.wind_speeds_full(reversed_layout, [0.0, 10.0])
    assert [t[1] for t in turbines_sorted] == [0.0, 882.0, 1764.0]
    assert velocities == [10.0, 7.71, 6.13]


def test_yaw_is_symmetric_north_south(model):
    """A southerly wind produces exactly what a northerly one does.

    Upstream zeroes the farm for any wind without a northward component; this
    port takes the absolute projection instead, because LOTUSim's wind
    direction is driven freely from the Unity sliders.
    """
    north = model.wind_speeds_full(LAYOUT_A, [0.0, 10.0])[1]
    south = model.wind_speeds_full(LAYOUT_A, [0.0, -10.0])[1]
    assert sorted(north) == sorted(south)
    assert max(south) > 0.0


def test_crosswind_row_is_unwaked(model):
    """Turbines side by side across the wind do not shadow each other."""
    row = [(0.0, 0.0, HUB), (882.0, 0.0, HUB), (1764.0, 0.0, HUB)]
    _, velocities, _ = model.wind_speeds_full(row, [0.0, 10.0])
    assert velocities == [10.0, 10.0, 10.0]


def test_z_is_the_vertical_axis(model):
    """ENU convention: two turbines at the same x/y but different z are
    stacked vertically, not spread downwind, so neither wakes the other."""
    stacked = [(0.0, 0.0, HUB), (0.0, 0.0, HUB + 400.0)]
    _, velocities, _ = model.wind_speeds_full(stacked, [0.0, 10.0])
    assert velocities == [10.0, 10.0]


def test_power_is_zero_outside_the_operating_range(model):
    assert model.power(2.9, hub_height=HUB) == 0.0
    assert model.power(25.1, hub_height=HUB) == 0.0
    assert model.power(10.0, hub_height=HUB) > 0.0


def test_hub_height_shifts_power_through_shear(model):
    """Shear is resolved over the rotor disk, so hub height changes the answer.

    A lower hub sits deeper in the boundary layer, where the disk-averaged
    speed falls further below the hub-height value.
    """
    low = model.power(10.0, hub_height=70.0)
    high = model.power(10.0, hub_height=120.0)
    assert low < high
    # Both stay within a few percent of the unsheared cubic — shear corrects
    # the answer, it does not dominate it.
    unsheared = 0.5 * 1.225 * np.pi * 63.0 ** 2 * 0.498 * 10.0 ** 3
    assert 0.9 < low / unsheared < 1.0


def test_zero_wind_is_rejected(model):
    with pytest.raises(ValueError):
        model.wind_speeds_full(LAYOUT_A, [0.0, 0.0])


def test_layout_shape_is_validated(model):
    with pytest.raises(ValueError):
        model.wind_speeds_full([(0.0, 0.0)], [0.0, 10.0])


def test_rotor_disk_reaching_the_ground_does_not_produce_nan():
    """A hub lower than the rotor radius must not sample negative altitudes."""
    model = LarsenWakeModel(diameter=126.0, ct=0.75, cp=0.498, ambient_ti=0.08)
    power = model.power(10.0, hub_height=20.0)
    assert np.isfinite(power) and power > 0.0
