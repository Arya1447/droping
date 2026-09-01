import config
from prediction import PredictionStabilityTracker, predict_drop_point


def _base_target():
    return {
        "lat": -6.895, "lon": 107.605, "alt_m": None,
        "box": {"north_m": 5.0, "south_m": 5.0, "east_m": 5.0, "west_m": 5.0},
        "required_waypoint": 6, "servo_channel": 7, "release_pwm": 2100,
        "max_cross_track_error_m": 10.0,
    }


def test_predict_drop_point_end_to_end_test_case():
    target = _base_target()
    tracker = PredictionStabilityTracker(stable_cycles=1, max_change_m=1000.0)

    result = predict_drop_point(
        payload_id=1, payload_mass_kg=0.5, mass_source="CONFIGURED", mass_uncertainty_kg=0.005,
        aircraft_lat=-6.9, aircraft_lon=107.6, origin_lat=-6.9, origin_lon=107.6,
        target=target,
        altitude_raw_m=100.0, altitude_filtered_m=100.0, altitude_source="TEST", altitude_valid=True,
        ground_velocity_n=18.0, ground_velocity_e=0.0, ground_velocity_u=0.0,
        air_speed_mps=17.0, heading_deg=0.0, flight_path_angle_deg=0.0,
        wind_speed_mps=5.0, wind_direction_from_deg=180.0, wind_source="ESTIMATED", wind_quality="OK",
        servo_delay_s=0.3,
        waypoint_passed=True, gps_valid=True, ekf_valid=True, ground_speed_valid=True,
        airspeed_valid=True, heartbeat_valid=True, telemetry_health="HEALTHY",
        servo_mapping_valid=True, servo_safety_valid=True, live_release_enabled=False,
        already_released=False, stability_tracker=tracker,
    )

    assert result.predicted_fall_time_s > 0
    assert result.predicted_drop_distance_m > 0
    assert result.servo_displacement_m == 18.0 * 0.3
    assert result.target_valid is True
    # DRY_RUN (live_release_enabled=False) always blocks release
    assert result.release_allowed is False
    assert "LIVE_RELEASE_DISABLED" in result.release_block_reason


def test_predict_drop_point_blocks_on_stale_gps():
    target = _base_target()
    result = predict_drop_point(
        payload_id=1, payload_mass_kg=0.5, mass_source="CONFIGURED", mass_uncertainty_kg=0.005,
        aircraft_lat=-6.9, aircraft_lon=107.6, origin_lat=-6.9, origin_lon=107.6,
        target=target,
        altitude_raw_m=100.0, altitude_filtered_m=100.0, altitude_source="TEST", altitude_valid=True,
        ground_velocity_n=18.0, ground_velocity_e=0.0, ground_velocity_u=0.0,
        servo_delay_s=0.3,
        waypoint_passed=True, gps_valid=False, ekf_valid=True, ground_speed_valid=True,
        airspeed_valid=True, heartbeat_valid=True, telemetry_health="CRITICAL",
        servo_mapping_valid=True, servo_safety_valid=True, live_release_enabled=True,
    )
    assert result.release_allowed is False
    assert "GPS_INVALID" in result.release_block_reason


def test_predict_drop_point_invalid_target():
    target = _base_target()
    target["lat"] = None
    target["lon"] = None
    result = predict_drop_point(
        payload_id=1, payload_mass_kg=0.5, mass_source="CONFIGURED", mass_uncertainty_kg=0.005,
        aircraft_lat=-6.9, aircraft_lon=107.6, origin_lat=-6.9, origin_lon=107.6,
        target=target,
        altitude_raw_m=100.0, altitude_filtered_m=100.0, altitude_source="TEST", altitude_valid=True,
        ground_velocity_n=18.0, ground_velocity_e=0.0, ground_velocity_u=0.0,
        servo_delay_s=0.3,
    )
    assert result.target_valid is False
    assert "TARGET_INVALID" in result.release_block_reason


def test_target_altitude_offset_defaults_to_level_terrain():
    target = _base_target()  # alt_m=None
    result = predict_drop_point(
        payload_id=1, payload_mass_kg=0.5, mass_source="CONFIGURED", mass_uncertainty_kg=0.005,
        aircraft_lat=-6.9, aircraft_lon=107.6, origin_lat=-6.9, origin_lon=107.6,
        target=target,
        altitude_raw_m=100.0, altitude_filtered_m=100.0, altitude_source="TEST", altitude_valid=True,
        ground_velocity_n=18.0, ground_velocity_e=0.0, ground_velocity_u=0.0,
        servo_delay_s=0.3,
    )
    assert result.target_altitude_source == "ASSUMED_LEVEL_TERRAIN"
    assert result.target_altitude_offset_m == 0.0
    assert result.effective_drop_height_m == 100.0


def test_target_altitude_offset_reduces_effective_drop_height():
    """Target sitting 20 m higher than home -> effective drop height and
    flight time both shrink relative to the flat-terrain case.
    """
    target = _base_target()
    target["alt_m"] = 20.0
    result = predict_drop_point(
        payload_id=1, payload_mass_kg=0.5, mass_source="CONFIGURED", mass_uncertainty_kg=0.005,
        aircraft_lat=-6.9, aircraft_lon=107.6, origin_lat=-6.9, origin_lon=107.6,
        target=target,
        altitude_raw_m=100.0, altitude_filtered_m=100.0, altitude_source="TEST", altitude_valid=True,
        ground_velocity_n=18.0, ground_velocity_e=0.0, ground_velocity_u=0.0,
        servo_delay_s=0.3,
    )
    assert result.target_altitude_source == "CONFIGURED"
    assert result.target_altitude_offset_m == 20.0
    assert result.effective_drop_height_m == 80.0
    assert result.predicted_fall_time_s < 4.51  # sqrt(2*100/9.81) baseline


def test_geofence_status_passed_through_to_result():
    target = _base_target()
    result = predict_drop_point(
        payload_id=1, payload_mass_kg=0.5, mass_source="CONFIGURED", mass_uncertainty_kg=0.005,
        aircraft_lat=-6.9, aircraft_lon=107.6, origin_lat=-6.9, origin_lon=107.6,
        target=target,
        altitude_raw_m=100.0, altitude_filtered_m=100.0, altitude_source="TEST", altitude_valid=True,
        ground_velocity_n=18.0, ground_velocity_e=0.0, ground_velocity_u=0.0,
        servo_delay_s=0.3,
        geofence_inside=True, geofence_source="CONFIGURED",
    )
    assert result.geofence_inside is True
    assert result.geofence_source == "CONFIGURED"


def test_geofence_outside_blocks_release():
    target = _base_target()
    result = predict_drop_point(
        payload_id=1, payload_mass_kg=0.5, mass_source="CONFIGURED", mass_uncertainty_kg=0.005,
        aircraft_lat=-6.9, aircraft_lon=107.6, origin_lat=-6.9, origin_lon=107.6,
        target=target,
        altitude_raw_m=100.0, altitude_filtered_m=100.0, altitude_source="TEST", altitude_valid=True,
        ground_velocity_n=18.0, ground_velocity_e=0.0, ground_velocity_u=0.0,
        servo_delay_s=0.3,
        waypoint_passed=True, gps_valid=True, ekf_valid=True, ground_speed_valid=True,
        airspeed_valid=True, heartbeat_valid=True, telemetry_health="HEALTHY",
        servo_mapping_valid=True, servo_safety_valid=True, live_release_enabled=True,
        geofence_inside=False, geofence_source="CONFIGURED",
    )
    assert result.release_allowed is False
    assert "OUTSIDE_GEOFENCE" in result.release_block_reason


def test_target_elevation_above_aircraft_blocks_without_crashing():
    target = _base_target()
    target["alt_m"] = 150.0  # higher than the aircraft's own altitude
    result = predict_drop_point(
        payload_id=1, payload_mass_kg=0.5, mass_source="CONFIGURED", mass_uncertainty_kg=0.005,
        aircraft_lat=-6.9, aircraft_lon=107.6, origin_lat=-6.9, origin_lon=107.6,
        target=target,
        altitude_raw_m=100.0, altitude_filtered_m=100.0, altitude_source="TEST", altitude_valid=True,
        ground_velocity_n=18.0, ground_velocity_e=0.0, ground_velocity_u=0.0,
        servo_delay_s=0.3,
    )
    assert result.effective_drop_height_m == -50.0
    assert result.release_allowed is False
    assert "TARGET_ELEVATION_ABOVE_AIRCRAFT" in result.release_block_reason
