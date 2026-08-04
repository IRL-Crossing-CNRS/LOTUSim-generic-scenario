"""Regression tests for the Blended wake model (spatial wake field)."""

import pytest

from lotusim_sdk.agents.environment.wake.blended import BlendedWakeModel
from lotusim_sdk.agents.environment.wake.larsen import LarsenWakeModel

NREL_5MW = dict(diameter=126.0, ct=0.75, cp=0.498, air_density=1.225, ambient_ti=0.08)


@pytest.fixture
def model():
    return BlendedWakeModel(LarsenWakeModel(**NREL_5MW))


def test_no_deficit_upstream_or_at_rotor(model):
    assert model.velocity_at_point(10.0, 0.0, 0.0) == 10.0
    assert model.velocity_at_point(10.0, -50.0, 0.0) == 10.0


def test_deficit_deepest_on_centreline(model):
    on_axis = model.velocity_at_point(10.0, 500.0, 0.0)
    off_axis = model.velocity_at_point(10.0, 500.0, 100.0)
    assert on_axis < off_axis < 10.0


def test_deficit_recovers_far_downstream(model):
    near = 10.0 - model.velocity_at_point(10.0, 200.0, 0.0)
    far = 10.0 - model.velocity_at_point(10.0, 3000.0, 0.0)
    assert far < near


def test_blend_weight_favours_larsen_near_wake(model):
    assert model.blend_weight(0.5 * model.diameter) == 1.0
    assert model.blend_weight(50.0 * model.diameter) == pytest.approx(0.40, abs=1e-6)


def test_farm_velocity_at_point_matches_single_turbine_for_one_turbine(model):
    turbines = [(0.0, 0.0, 90.0)]
    direct = model.velocity_at_point(10.0, 500.0, 0.0)
    farm = model.farm_velocity_at_point(10.0, [0.0, 1.0], 0.0, 500.0, turbines)
    assert farm == pytest.approx(direct)


def test_farm_velocity_at_point_ignores_downstream_turbines(model):
    """A turbine query point upstream of every turbine sees no deficit."""
    turbines = [(0.0, 500.0, 90.0), (0.0, 1000.0, 90.0)]
    speed = model.farm_velocity_at_point(10.0, [0.0, 1.0], 0.0, 0.0, turbines)
    assert speed == 10.0


def test_farm_velocity_at_point_follows_arbitrary_wind_direction(model):
    """The query/turbine projection tracks the wind vector, not a fixed axis.

    Two turbines placed along the *east* axis with an eastward wind should
    show the same downstream deficit pattern a north-aligned layout shows
    for a northward wind — this is the point of projecting onto the current
    wind vector instead of assuming one fixed axis.
    """
    turbines_north = [(0.0, 0.0, 90.0)]
    turbines_east = [(0.0, 0.0, 90.0)]
    north = model.farm_velocity_at_point(10.0, [0.0, 1.0], 0.0, 500.0, turbines_north)
    east = model.farm_velocity_at_point(10.0, [1.0, 0.0], 500.0, 0.0, turbines_east)
    assert north == pytest.approx(east)


def test_calibrated_sigma_grows_with_distance(model):
    assert model.calibrated_sigma(100.0) < model.calibrated_sigma(5000.0)
