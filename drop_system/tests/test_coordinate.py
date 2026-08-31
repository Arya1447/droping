import math

import pytest

from coordinate_utils import (
    along_cross_track, bearing_deg, flight_path_angle_from_velocity,
    ground_track_from_velocity, horizontal_to_ne, latlon_to_local, local_to_latlon,
    wrap_angle_deg,
)
from wind_model import (
    consistency_residual, estimate_wind_vector, wind_speed_direction_from_vector,
    wind_vector_from_speed_direction,
)


def test_angle_wrapping():
    assert wrap_angle_deg(370.0) == pytest.approx(10.0)
    assert wrap_angle_deg(190) == pytest.approx(-170.0)
    assert wrap_angle_deg(-190) == pytest.approx(170.0)
    # heading 350 vs heading 340(-20 normalized): true difference is 10 deg
    assert wrap_angle_deg(350 - 340) == pytest.approx(10.0)


def test_ground_velocity_and_track():
    v_n, v_e = horizontal_to_ne(10.0, 90.0)
    assert v_n == pytest.approx(0.0, abs=1e-9)
    assert v_e == pytest.approx(10.0)
    chi = ground_track_from_velocity(v_n, v_e)
    assert chi == pytest.approx(90.0)


def test_heading_conversion_hdg_cdeg():
    hdg_cdeg = 4530
    heading_deg = hdg_cdeg / 100.0
    assert heading_deg == pytest.approx(45.3)


def test_flight_path_angle():
    gamma = flight_path_angle_from_velocity(v_up=2.0, v_horizontal=10.0)
    assert gamma == pytest.approx(math.degrees(math.atan2(2.0, 10.0)))
    assert flight_path_angle_from_velocity(0.0, 10.0) == pytest.approx(0.0)


def test_along_track_and_cross_track():
    # target due north of aircraft, track = north (chi=0)
    along, cross = along_cross_track(100.0, 0.0, 0.0)
    assert along == pytest.approx(100.0)
    assert cross == pytest.approx(0.0)

    # target due east, track = north
    along, cross = along_cross_track(0.0, 100.0, 0.0)
    assert along == pytest.approx(0.0, abs=1e-9)
    assert cross == pytest.approx(100.0)


def test_bearing():
    assert bearing_deg(100.0, 0.0) == pytest.approx(0.0)
    assert bearing_deg(0.0, 100.0) == pytest.approx(90.0)
    assert bearing_deg(-100.0, 0.0) == pytest.approx(180.0)


def test_latlon_local_roundtrip():
    origin_lat, origin_lon = -6.9, 107.6
    lat, lon = -6.895, 107.605
    n, e = latlon_to_local(lat, lon, origin_lat, origin_lon)
    lat2, lon2 = local_to_latlon(n, e, origin_lat, origin_lon)
    assert lat2 == pytest.approx(lat, abs=1e-9)
    assert lon2 == pytest.approx(lon, abs=1e-9)


def test_wind_vector_from_speed_direction_from_convention():
    # wind FROM north (0 deg) blows TOWARD south -> negative N component
    wn, we = wind_vector_from_speed_direction(5.0, 0.0)
    assert wn == pytest.approx(-5.0, abs=1e-9)
    assert we == pytest.approx(0.0, abs=1e-9)


def test_wind_vector_roundtrip():
    speed, from_deg = wind_speed_direction_from_vector(*wind_vector_from_speed_direction(7.0, 123.0))
    assert speed == pytest.approx(7.0)
    assert from_deg == pytest.approx(123.0)


def test_estimate_wind_vector_ground_minus_air():
    # V_ground = V_air + V_wind  =>  V_wind = V_ground - V_air
    ground = (18.0, 0.0, 0.0)
    air = (17.0, 0.0, 0.0)
    wind = estimate_wind_vector(*ground, *air)
    assert wind == pytest.approx((1.0, 0.0, 0.0))


def test_estimate_wind_vector_unknown_without_air_direction():
    assert estimate_wind_vector(18.0, 0.0, 0.0, None, None, None) is None


def test_consistency_residual_zero_when_consistent():
    r = consistency_residual(18.0, 0.0, 0.0, 17.0, 0.0, 0.0, 1.0, 0.0, 0.0)
    assert r == pytest.approx(0.0, abs=1e-9)


def test_consistency_residual_nonzero_when_inconsistent():
    r = consistency_residual(18.0, 0.0, 0.0, 17.0, 0.0, 0.0, 0.5, 0.0, 0.0)
    assert r == pytest.approx(0.5, abs=1e-9)
