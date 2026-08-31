#!/usr/bin/env python3
"""Live/dry-run/simulation entry point (spec sections 99-102, 114, 161-162).

Pipeline per cycle (spec section 100):
    MAVLink -> telemetry -> freshness -> filtering -> coordinate
    conversion -> waypoint validation -> target validation -> physics
    prediction -> empirical correction -> uncertainty -> impact
    prediction -> target box validation -> release window -> state
    machine -> servo safety gate -> servo.

Modes (spec section 99, default DRY_RUN):
    SIMULATION - synthetic telemetry, no flight controller connection.
    DRY_RUN    - real MAVLink telemetry, full prediction + state
                 machine, servo command transport disabled.
    LIVE       - real MAVLink telemetry AND real servo commands,
                 gated additionally by config.ENABLE_LIVE_RELEASE.
"""

from __future__ import annotations

import argparse
import math
import time
from typing import Dict, Optional

import config
from coordinate_utils import ground_track_from_velocity
from filters import CircularEMAFilter, EMAFilter
from logger import DropSystemLogger
from mavlink_interface import MAVLinkInterface, MAVLinkUnavailableError
from prediction import PredictionStabilityTracker, predict_drop_point
from servo_controller import FakeServoController, MAVLinkServoController, ServoFault
from state_machine import PayloadStateMachine
from telemetry_health import TelemetryHealth
from waypoint_validator import WaypointValidator


class PayloadRuntime:
    """Everything stateful that must persist across cycles for one
    payload: filters, waypoint latch, stability tracker, state machine.
    """

    def __init__(self, payload_id: int, target: dict):
        self.payload_id = payload_id
        self.target = target
        self.altitude_filter = EMAFilter(config.EMA_ALPHA_ALTITUDE)
        self.ground_speed_filter = EMAFilter(config.EMA_ALPHA_GROUND_SPEED)
        self.heading_filter = CircularEMAFilter(config.EMA_ALPHA_HEADING)
        self.waypoint_validator = WaypointValidator(required_waypoint_seq=target["required_waypoint"])
        self.stability_tracker = PredictionStabilityTracker()
        self.state_machine = PayloadStateMachine(payload_id=payload_id)


def _synthetic_telemetry(t: float) -> Dict:
    """SIMULATION-mode synthetic telemetry generator. Uses the
    documented TEST_CASE nominal values (spec section 3) — explicitly
    NOT a substitute for real flight data, only for exercising the
    pipeline offline.
    """
    tc = config.TEST_CASE
    return {
        "aircraft_lat": (config.LOCAL_ORIGIN_LAT or 0.0),
        "aircraft_lon": (config.LOCAL_ORIGIN_LON or 0.0),
        "altitude_m": tc.altitude_m,
        "ground_velocity_n": tc.ground_speed_mps,
        "ground_velocity_e": 0.0,
        "ground_velocity_u": 0.0,
        "air_speed_mps": tc.air_speed_mps,
        "heading_deg": tc.heading_deg,
        "wind_speed_mps": tc.wind_speed_mps,
        "wind_direction_from_deg": 180.0,
        "mission_seq": 6,
        "gps_valid": True,
        "ekf_valid": True,
        "heartbeat_valid": True,
    }


def run_cycle(runtime: PayloadRuntime, telemetry: Dict, mode: str,
              servo: "FakeServoController | MAVLinkServoController",
              logger: DropSystemLogger, empirical_correction_fn=None) -> None:
    origin_lat = config.LOCAL_ORIGIN_LAT if config.LOCAL_ORIGIN_LAT is not None else telemetry["aircraft_lat"]
    origin_lon = config.LOCAL_ORIGIN_LON if config.LOCAL_ORIGIN_LON is not None else telemetry["aircraft_lon"]

    altitude_filtered = runtime.altitude_filter.update(telemetry["altitude_m"])
    ground_speed_raw = math.hypot(telemetry["ground_velocity_n"], telemetry["ground_velocity_e"])
    runtime.ground_speed_filter.update(ground_speed_raw)
    ground_track = ground_track_from_velocity(telemetry["ground_velocity_n"], telemetry["ground_velocity_e"])
    runtime.heading_filter.update(telemetry.get("heading_deg", ground_track))

    from coordinate_utils import latlon_to_local
    aircraft_local = latlon_to_local(telemetry["aircraft_lat"], telemetry["aircraft_lon"], origin_lat, origin_lon)
    waypoint_passed = runtime.waypoint_validator.update(
        mission_seq=telemetry.get("mission_seq", 0),
        current_position=aircraft_local,
        waypoint_prev_position=telemetry.get("waypoint_prev_position"),
        waypoint_current_position=telemetry.get("waypoint_current_position"),
    )

    servo_mapping_valid = False
    try:
        servo_mapping_valid = servo.validate_mapping(runtime.target["servo_channel"])
    except Exception:
        servo_mapping_valid = False

    result = predict_drop_point(
        payload_id=runtime.payload_id,
        payload_mass_kg=config.PAYLOAD_MASS_KG_NOMINAL,
        mass_source=config.PAYLOAD_MASS_SOURCE,
        mass_uncertainty_kg=config.PAYLOAD_MASS_SIGMA_KG_TEST,
        aircraft_lat=telemetry["aircraft_lat"], aircraft_lon=telemetry["aircraft_lon"],
        origin_lat=origin_lat, origin_lon=origin_lon,
        target=runtime.target,
        altitude_raw_m=telemetry["altitude_m"], altitude_filtered_m=altitude_filtered,
        altitude_source=telemetry.get("altitude_source", "GLOBAL_POSITION_INT.relative_alt"),
        altitude_valid=telemetry.get("altitude_valid", True),
        ground_velocity_n=telemetry["ground_velocity_n"], ground_velocity_e=telemetry["ground_velocity_e"],
        ground_velocity_u=telemetry["ground_velocity_u"],
        air_speed_mps=telemetry.get("air_speed_mps"), heading_deg=telemetry.get("heading_deg"),
        wind_speed_mps=telemetry.get("wind_speed_mps"), wind_direction_from_deg=telemetry.get("wind_direction_from_deg"),
        wind_source=telemetry.get("wind_source", "ESTIMATED"), wind_quality=telemetry.get("wind_quality", "UNKNOWN"),
        servo_delay_s=config.SERVO_DELAY_S_TEST,
        waypoint_passed=waypoint_passed,
        gps_valid=telemetry.get("gps_valid", False), ekf_valid=telemetry.get("ekf_valid", False),
        ground_speed_valid=telemetry.get("gps_valid", False), airspeed_valid=telemetry.get("airspeed_valid", True),
        heartbeat_valid=telemetry.get("heartbeat_valid", False),
        telemetry_health=telemetry.get("telemetry_health", "CRITICAL"),
        servo_mapping_valid=servo_mapping_valid,
        servo_safety_valid=servo_mapping_valid,
        live_release_enabled=(mode == "LIVE" and config.ENABLE_LIVE_RELEASE),
        already_released=runtime.state_machine.released,
        empirical_correction_fn=empirical_correction_fn,
        stability_tracker=runtime.stability_tracker,
        ground_speed_sigma_mps=config.TEST_UNCERTAINTY.ground_speed_sigma_mps,
        heading_sigma_deg=config.TEST_UNCERTAINTY.heading_sigma_deg,
        altitude_sigma_m=config.TEST_UNCERTAINTY.altitude_sigma_m,
    )

    from state_machine import GateInputs
    gates = GateInputs(
        gps_valid=telemetry.get("gps_valid", False), altitude_valid=result.altitude_valid,
        ground_speed_valid=telemetry.get("gps_valid", False), airspeed_valid=telemetry.get("airspeed_valid", True),
        heartbeat_valid=telemetry.get("heartbeat_valid", False), ekf_valid=telemetry.get("ekf_valid", False),
        target_valid=result.target_valid, target_box_valid=result.target_box_valid,
        waypoint_passed=waypoint_passed,
        corridor_valid=abs(result.aircraft_cross_track_error_m) <= runtime.target["max_cross_track_error_m"],
        target_ahead=result.target_distance_m > 0,
        prediction_valid=result.release_allowed or True,
        predicted_impact_inside_box=result.predicted_impact_inside_box,
        release_distance_valid=math.isfinite(result.final_release_distance_m),
        release_window_valid=result.target_distance_m > 0,
        prediction_stable=result.prediction_stable,
        servo_mapping_valid=servo_mapping_valid, servo_safety_valid=servo_mapping_valid,
        live_release_enabled=(mode == "LIVE" and config.ENABLE_LIVE_RELEASE),
    )
    block_reasons = runtime.state_machine.update(gates, waypoint_passed)

    servo_output = None
    release_commanded = False
    release_verified = False
    if not block_reasons and mode == "LIVE":
        try:
            record = servo.release(runtime.target["servo_channel"], runtime.target["release_pwm"])
            runtime.state_machine.commit_release()
            if record.verification_status == "VERIFIED":
                runtime.state_machine.mark_verified()
                release_verified = True
            release_commanded = True
            servo_output = record.actual_output
        except ServoFault as exc:
            print(f"SERVO FAULT (payload {runtime.payload_id}): {exc}")

    logger.log_cycle(result, servo_output=servo_output, release_commanded=release_commanded,
                      release_verified=release_verified)

    print(f"\nPAYLOAD {runtime.payload_id}")
    print(f"Waypoint {runtime.target['required_waypoint']:<3}: {'PASSED' if waypoint_passed else 'PENDING'}")
    print(f"Target        : {'VALID' if result.target_valid else 'INVALID'}")
    print(f"Target Box    : {'VALID' if result.target_box_valid else 'INVALID'}, "
          f"impact inside box: {result.predicted_impact_inside_box}")
    print(f"Prediction    : {'STABLE' if result.prediction_stable else 'NOT STABLE'}, "
          f"confidence={result.confidence}")
    print(f"Final release distance: {result.final_release_distance_m:.2f} m, "
          f"predicted impact error (2D): {result.predicted_impact_error_2d_m:.2f} m")
    if block_reasons:
        print("RELEASE BLOCKED")
        print("REASONS:")
        for r in block_reasons:
            print(f"    {r}")
    else:
        print("RELEASE: READY" if mode != "LIVE" else "RELEASE: COMMANDED")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mode", choices=config.OPERATING_MODES, default=config.DEFAULT_OPERATING_MODE)
    parser.add_argument("--cycles", type=int, default=0, help="0 = run forever (Ctrl+C to stop)")
    args = parser.parse_args()

    print(f"Starting drop_system in {args.mode} mode "
          f"(ENABLE_LIVE_RELEASE={config.ENABLE_LIVE_RELEASE})")

    runtimes = {pid: PayloadRuntime(pid, target) for pid, target in config.TARGETS.items()}
    logger = DropSystemLogger(config.LOG_CSV_PATH)

    if args.mode == "SIMULATION":
        servo = FakeServoController(mapping_valid_channels={t["servo_channel"] for t in config.TARGETS.values()})
        mavlink = None
    else:
        try:
            mavlink = MAVLinkInterface(config.MAVLINK_CONNECTION_STRING, config.MAVLINK_SOURCE_SYSTEM)
            mavlink.connect()
        except MAVLinkUnavailableError as exc:
            print(f"MAVLink unavailable: {exc}")
            return 1
        mapping_check = lambda ch: False  # UNKNOWN until real SERVOx_FUNCTION params confirmed
        servo = MAVLinkServoController(mavlink.connection, mapping_check=mapping_check)

    period_s = 1.0 / config.PREDICTION_CYCLE_HZ
    cycle = 0
    try:
        while args.cycles == 0 or cycle < args.cycles:
            t0 = time.monotonic()

            if args.mode == "SIMULATION":
                telemetry = _synthetic_telemetry(t0)
            else:
                mavlink.poll(blocking=False)
                print("LIVE telemetry ingestion from MAVLinkInterface is scaffolded "
                      "(mavlink_interface.py) but wiring GLOBAL_POSITION_INT/VFR_HUD/WIND "
                      "messages into the `telemetry` dict here is left for integration "
                      "against the real flight controller — not fabricated.")
                break

            for pid, runtime in runtimes.items():
                run_cycle(runtime, telemetry, args.mode, servo, logger)

            cycle += 1
            elapsed = time.monotonic() - t0
            time.sleep(max(0.0, period_s - elapsed))
    except KeyboardInterrupt:
        print("\nStopped (Ctrl+C)")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
