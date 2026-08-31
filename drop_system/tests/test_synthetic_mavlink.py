import pytest

from mavlink_mapping import MAPPING_TABLE


def test_mapping_table_covers_required_variables():
    variables = {entry.variable for entry in MAPPING_TABLE}
    required = {
        "latitude_deg", "longitude_deg", "altitude_m",
        "velocity_north_mps", "velocity_east_mps",
        "heading_deg", "airspeed_mps", "mission_current_seq", "servo_output_us",
    }
    assert required.issubset(variables)


def test_mapping_conversions_are_correct():
    by_var = {e.variable: e for e in MAPPING_TABLE}
    assert by_var["altitude_m"].conversion(1000) == 1.0  # mm -> m
    assert by_var["velocity_north_mps"].conversion(100) == 1.0  # cm/s -> m/s
    assert by_var["heading_deg"].conversion(100) == 1.0  # cdeg -> deg


def test_synthetic_mavlink_pipeline_end_to_end():
    """spec section 109: generate synthetic GLOBAL_POSITION_INT-shaped
    data and confirm it flows through the mapping conversions correctly,
    without requiring a real pymavlink connection.
    """
    synthetic_global_position_int = {
        "lat": int(-6.9 * 1e7), "lon": int(107.6 * 1e7),
        "relative_alt": 100000,  # mm
        "vx": 1800, "vy": 0, "vz": 0,  # cm/s
        "hdg": 0,
    }
    by_var = {e.variable: e for e in MAPPING_TABLE}
    lat = by_var["latitude_deg"].conversion(synthetic_global_position_int["lat"])
    lon = by_var["longitude_deg"].conversion(synthetic_global_position_int["lon"])
    alt = by_var["altitude_m"].conversion(synthetic_global_position_int["relative_alt"])
    vn = by_var["velocity_north_mps"].conversion(synthetic_global_position_int["vx"])

    assert lat == pytest.approx(-6.9)
    assert lon == pytest.approx(107.6)
    assert alt == pytest.approx(100.0)
    assert vn == pytest.approx(18.0)
