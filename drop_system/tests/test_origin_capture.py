"""Tests for the auto-capture-local-origin-from-first-GPS-fix behavior
(main.py's `_maybe_capture_origin`), added so DRY_RUN/LIVE don't
require a manually-configured `config.LOCAL_ORIGIN_LAT/LON` before
they'll even start — instead they HOLD until the aircraft's own first
valid GPS fix, then lock that position in as the origin for the rest
of the run.
"""

from main import _maybe_capture_origin


def test_no_capture_when_gps_invalid():
    telemetry = {"gps_valid": False, "aircraft_lat": -6.9, "aircraft_lon": 107.6}
    assert _maybe_capture_origin(telemetry) is None


def test_no_capture_when_position_missing():
    telemetry = {"gps_valid": True}  # GLOBAL_POSITION_INT hasn't arrived yet
    assert _maybe_capture_origin(telemetry) is None


def test_no_capture_when_gps_valid_key_absent():
    telemetry = {"aircraft_lat": -6.9, "aircraft_lon": 107.6}
    assert _maybe_capture_origin(telemetry) is None


def test_captures_when_gps_valid_and_position_present():
    telemetry = {"gps_valid": True, "aircraft_lat": -6.9123456, "aircraft_lon": 107.6123456}
    result = _maybe_capture_origin(telemetry)
    assert result == (-6.9123456, 107.6123456)
