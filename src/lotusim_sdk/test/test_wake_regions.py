"""Tests for turning the wake field into WindRegion cone segments."""

import math

import pytest

from lotusim_sdk.agents.environment.wake.blended import BlendedWakeModel
from lotusim_sdk.agents.environment.wake.larsen import LarsenWakeModel
from lotusim_sdk.agents.environment.wake.wake_regions import (
    WakeRegionGenerator,
    wind_changed_enough,
)

NREL_5MW = dict(diameter=126.0, ct=0.75, cp=0.498, air_density=1.225, ambient_ti=0.08)


@pytest.fixture
def generator():
    blended = BlendedWakeModel(LarsenWakeModel(**NREL_5MW))
    return WakeRegionGenerator(blended, cell_diameters=0.5, deficit_threshold=0.05,
                                max_downstream_diameters=15.0)


def test_template_is_nonempty_and_bounded(generator):
    assert generator._template
    d = generator._model.diameter
    for x_start, x_end, r_start, r_end in generator._template:
        assert x_end > x_start
        assert r_start > 0.0
        assert r_end > 0.0
        assert x_end <= 15.0 * d + 1e-6


def test_template_first_slice_starts_at_rotor_radius(generator):
    """The first slice starts right at the rotor disk, so its r_start should
    be exactly the rotor radius — not a bisected value (the deficit formula
    is 0 everywhere at x=0, which would make a bisection meaningless)."""
    _, _, r_start, _ = generator._template[0]
    radius = generator._model.diameter / 2.0
    assert r_start == pytest.approx(radius)


def test_template_chains_continuously(generator):
    """Consecutive slices should share a boundary radius, so the geometry is
    one continuous tapered cone rather than independently bisected steps."""
    for (_, _, _, r_end), (_, _, r_start_next, _) in zip(
        generator._template, generator._template[1:]
    ):
        assert r_start_next == pytest.approx(r_end)


def test_single_turbine_regions_cover_downstream_axis(generator):
    turbines = [("t1", 0.0, 0.0, 90.0)]
    regions = generator.regions_for_wind(turbines, [0.0, 10.0])
    assert regions
    for r in regions:
        assert r["id"].startswith("wake_t1_")
        assert r["length"] > 0.0
        assert r["r_start"] > 0.0
        assert r["r_end"] > 0.0
        # Wind blows north (+y): every segment should sit downstream of the
        # rotor, with its axis pointing the same way.
        assert r["origin_y"] >= 0.0
        assert r["axis_y"] == pytest.approx(1.0)
        assert r["axis_x"] == pytest.approx(0.0)


def test_regions_follow_wind_direction(generator):
    turbines = [("t1", 0.0, 0.0, 90.0)]
    north = generator.regions_for_wind(turbines, [0.0, 10.0])
    east = generator.regions_for_wind(turbines, [10.0, 0.0])
    assert north and east
    # Northward wind: segments' axis points along +y. Eastward: along +x.
    assert all(r["axis_y"] == pytest.approx(1.0) for r in north)
    assert all(r["axis_x"] == pytest.approx(1.0) for r in east)


def test_region_velocity_points_downwind_and_is_deficient(generator):
    turbines = [("t1", 0.0, 0.0, 90.0)]
    regions = generator.regions_for_wind(turbines, [0.0, 10.0])
    for r in regions:
        assert r["vx"] == pytest.approx(0.0, abs=1e-9)
        assert 0.0 < r["vy"] < 10.0
        assert r["vz"] == 0.0


def test_no_regions_for_zero_wind(generator):
    turbines = [("t1", 0.0, 0.0, 90.0)]
    assert generator.regions_for_wind(turbines, [0.0, 0.0]) == []


def test_downstream_turbine_gets_fewer_or_no_new_regions_where_already_waked(generator):
    """A turbine sitting deep in another's wake starts from an already-reduced
    freestream, so compounded deficit crosses the drop threshold sooner —
    the second turbine should not contribute more slices than the first."""
    turbines = [("t1", 0.0, 0.0, 90.0), ("t2", 0.0, 300.0, 90.0)]
    regions = generator.regions_for_wind(turbines, [0.0, 10.0])
    n_t1 = sum(1 for r in regions if r["id"].startswith("wake_t1_"))
    n_t2 = sum(1 for r in regions if r["id"].startswith("wake_t2_"))
    assert n_t1 >= n_t2 >= 0


class TestWindChangedEnough:
    def test_first_call_always_true(self):
        assert wind_changed_enough(None, [0.0, 10.0], 5.0, 0.5) is True

    def test_tiny_change_is_false(self):
        assert wind_changed_enough([0.0, 10.0], [0.0, 10.05], 5.0, 0.5) is False

    def test_speed_change_beyond_threshold_is_true(self):
        assert wind_changed_enough([0.0, 10.0], [0.0, 10.6], 5.0, 0.5) is True

    def test_direction_change_beyond_threshold_is_true(self):
        angle = math.radians(6.0)
        rotated = [10.0 * math.sin(angle), 10.0 * math.cos(angle)]
        assert wind_changed_enough([0.0, 10.0], rotated, 5.0, 0.5) is True

    def test_direction_change_below_threshold_is_false(self):
        angle = math.radians(2.0)
        rotated = [10.0 * math.sin(angle), 10.0 * math.cos(angle)]
        assert wind_changed_enough([0.0, 10.0], rotated, 5.0, 0.5) is False

    def test_wind_starting_from_calm_is_true(self):
        assert wind_changed_enough([0.0, 0.0], [0.0, 3.0], 5.0, 0.5) is True

    def test_wind_dying_to_calm_is_true(self):
        assert wind_changed_enough([0.0, 3.0], [0.0, 0.0], 5.0, 0.5) is True
