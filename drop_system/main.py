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
from typing import Dict, Optional, Tuple

import config
import geofence
from coordinate_utils import ground_track_from_velocity, horizontal_to_ne, latlon_to_local
from filters import CircularEMAFilter, EMAFilter
from logger import DropSystemLogger
from mavlink_interface import (
    MAVLinkInterface, MAVLinkUnavailableError,
    ekf_valid_from_flags, gps_valid_from_fix_type,
)
from prediction import PredictionStabilityTracker, predict_drop_point
from servo_controller import FakeServoController, MAVLinkServoController, ServoFault
from state_machine import GateInputs, PayloadStateMachine
from telemetry_health import TelemetryHealth
from waypoint_validator import WaypointValidator
from wind_model import estimate_wind_vector, wind_speed_direction_from_vector

GPS_HDG_UNKNOWN = 65535  # MAVLink common.xml: GLOBAL_POSITION_INT.hdg sentinel for "unknown"


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
        # Local (N, E) position of the waypoint immediately before, and
        # of, this payload's required waypoint — resolved once at
        # startup from the real onboard mission (see
        # _resolve_waypoint_positions), not re-fetched every cycle.
        self.waypoint_prev_local: Optional[Tuple[float, float]] = None
        self.waypoint_current_local: Optional[Tuple[float, float]] = None


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
        "ground_speed_valid": True,
    }


def _resolve_waypoint_positions(runtimes: Dict[int, "PayloadRuntime"],
                                 mission_items: Dict[int, Tuple[float, float]],
                                 origin_lat: float, origin_lon: float) -> None:
    """Convert each payload's required-waypoint and previous-waypoint
    lat/lon (from the real onboard mission) into local (N, E), once.
    Missing entries stay None — waypoint_validator.py already handles a
    None waypoint position by simply not evaluating spatial crossing
    that cycle rather than crashing or guessing.
    """
    for runtime in runtimes.values():
        req = runtime.target["required_waypoint"]
        prev_ll = mission_items.get(req - 1)
        curr_ll = mission_items.get(req)
        runtime.waypoint_prev_local = (
            latlon_to_local(prev_ll[0], prev_ll[1], origin_lat, origin_lon) if prev_ll else None
        )
        runtime.waypoint_current_local = (
            latlon_to_local(curr_ll[0], curr_ll[1], origin_lat, origin_lon) if curr_ll else None
        )
        if runtime.waypoint_prev_local is None or runtime.waypoint_current_local is None:
            print(f"WARNING: payload {runtime.payload_id} required_waypoint={req} "
                  f"not found in the fetched mission (have items {sorted(mission_items)}) — "
                  "spatial waypoint-passed check will stay False until this is fixed "
                  "(config.py required_waypoint vs. the uploaded mission likely disagree).")


def _maybe_capture_origin(telemetry: Dict) -> Optional[Tuple[float, float]]:
    """Returns (lat, lon) to lock in as the local origin if this cycle's
    telemetry has a valid GPS fix and a position, else None. Pulled out
    as its own pure function so the capture DECISION is unit-testable
    without a real MAVLink connection.
    """
    if telemetry.get("gps_valid") and "aircraft_lat" in telemetry:
        return telemetry["aircraft_lat"], telemetry["aircraft_lon"]
    return None


def _live_telemetry(mavlink: MAVLinkInterface, health: TelemetryHealth, now: float) -> Dict:
    """Build one cycle's telemetry dict entirely from the latest received
    MAVLink messages (spec sections 145-165: LIVE altitude/velocity/
    heading/wind/waypoint state must come from real telemetry, never a
    hardcoded constant). Any field whose source message hasn't arrived
    yet is simply absent from the dict — run_cycle()/predict_drop_point()
    already default missing telemetry to fail-closed (False/None), never
    to a fabricated number.
    """
    telemetry: Dict = {}

    gpos = mavlink.get_latest("GLOBAL_POSITION_INT")
    if gpos is not None:
        m = gpos.msg
        health.touch("gps", True, now=gpos.timestamp)
        health.touch("altitude", True, now=gpos.timestamp)
        health.touch("ground_speed", True, now=gpos.timestamp)
        telemetry["aircraft_lat"] = m.lat / 1e7
        telemetry["aircraft_lon"] = m.lon / 1e7
        telemetry["altitude_m"] = m.relative_alt / 1000.0
        telemetry["altitude_source"] = "GLOBAL_POSITION_INT.relative_alt"
        telemetry["ground_velocity_n"] = m.vx / 100.0
        telemetry["ground_velocity_e"] = m.vy / 100.0
        telemetry["ground_velocity_u"] = -m.vz / 100.0  # MAVLink vz is down-positive
        if m.hdg != GPS_HDG_UNKNOWN:
            telemetry["heading_deg"] = m.hdg / 100.0

    vfr = mavlink.get_latest("VFR_HUD")
    if vfr is not None:
        health.touch("airspeed", True, now=vfr.timestamp)
        telemetry["air_speed_mps"] = vfr.msg.airspeed
        telemetry.setdefault("heading_deg", vfr.msg.heading)
        health.touch("heading", True, now=vfr.timestamp)

    wind = mavlink.get_latest("WIND")
    if wind is not None:
        health.touch("wind", True, now=wind.timestamp)
        telemetry["wind_speed_mps"] = wind.msg.speed
        telemetry["wind_direction_from_deg"] = wind.msg.direction
        telemetry["wind_source"] = "MAVLINK_WIND_ESTIMATE"
        telemetry["wind_quality"] = "OK"
    elif "air_speed_mps" in telemetry and "heading_deg" in telemetry and "ground_velocity_n" in telemetry:
        # No direct WIND message — fall back to the vector estimator,
        # which requires an air-relative direction. Heading is used as
        # that direction under an explicit ASSUMED-zero-sideslip
        # approximation (spec section 151: never silently assume this;
        # here it is labeled, not hidden).
        air_n, air_e = horizontal_to_ne(telemetry["air_speed_mps"], telemetry["heading_deg"])
        wind_vec = estimate_wind_vector(
            telemetry["ground_velocity_n"], telemetry["ground_velocity_e"], 0.0, air_n, air_e, 0.0
        )
        if wind_vec is not None:
            speed, direction = wind_speed_direction_from_vector(wind_vec[0], wind_vec[1])
            telemetry["wind_speed_mps"] = speed
            telemetry["wind_direction_from_deg"] = direction
            telemetry["wind_source"] = "ESTIMATED_ZERO_SIDESLIP_ASSUMED"
            telemetry["wind_quality"] = "DEGRADED"
    if "wind_source" not in telemetry:
        telemetry["wind_source"] = "UNAVAILABLE"
        telemetry["wind_quality"] = "UNKNOWN"

    mission_current = mavlink.get_latest("MISSION_CURRENT")
    if mission_current is not None:
        health.touch("mission", True, now=mission_current.timestamp)
        telemetry["mission_seq"] = mission_current.msg.seq
    else:
        telemetry["mission_seq"] = 0

    gps_raw = mavlink.get_latest("GPS_RAW_INT")
    telemetry["gps_valid"] = gps_valid_from_fix_type(gps_raw.msg.fix_type if gps_raw else None)

    ekf = mavlink.get_latest("EKF_STATUS_REPORT")
    telemetry["ekf_valid"] = ekf_valid_from_flags(ekf.msg.flags if ekf else None)

    heartbeat = mavlink.get_latest("HEARTBEAT")
    if heartbeat is not None:
        health.touch("heartbeat", True, now=heartbeat.timestamp)

    servo_feedback = mavlink.get_latest("SERVO_OUTPUT_RAW")
    if servo_feedback is not None:
        health.touch("servo_feedback", True, now=servo_feedback.timestamp)

    health_report = health.evaluate(now=now)
    telemetry["telemetry_health"] = health_report.state
    telemetry["altitude_valid"] = "altitude" not in health_report.critical_stale
    telemetry["airspeed_valid"] = "airspeed" not in health_report.critical_stale
    telemetry["ground_speed_valid"] = "ground_speed" not in health_report.critical_stale
    telemetry["heartbeat_valid"] = "heartbeat" not in health_report.critical_stale

    return telemetry


def _format_payload_detail(runtime: "PayloadRuntime", result, waypoint_passed: bool,
                            block_reasons: list) -> str:
    """Full per-payload parameter dump, as a single text block — written
    to config.LIVE_LOG_FILE (overwritten fresh each cycle), never to
    stdout, so the console stays quiet while every number is still
    visible in real time via `watch cat droping.log` (or similar).
    """
    def _fmt(value, unit="", digits=2):
        return f"{value:.{digits}f}{unit}" if value is not None else "N/A"

    lines = [
        f"=== PAYLOAD {runtime.payload_id} ===",
        f"Telemetry Health : {result.telemetry_health}",
        f"Geofence         : {'INSIDE' if result.geofence_inside else 'OUTSIDE'} (source={result.geofence_source})",
        f"Waypoint {runtime.target['required_waypoint']:<3}    : {'PASSED' if waypoint_passed else 'PENDING'}",
        f"Target           : {'VALID' if result.target_valid else 'INVALID'}",
        f"Target Box       : {'VALID' if result.target_box_valid else 'INVALID'}, "
        f"predicted impact inside box: {result.predicted_impact_inside_box}",
        f"Airspeed         : {_fmt(result.air_speed_mps, ' m/s')}",
        f"Ground Speed     : {_fmt(result.ground_speed_mps, ' m/s')} (ground_track={_fmt(result.ground_track_deg, ' deg')})",
        f"Altitude         : raw={_fmt(result.altitude_raw_m, ' m')}, "
        f"filtered={_fmt(result.altitude_filtered_m, ' m')} (source={result.altitude_source})",
        f"Distance-to-target (along-track): {_fmt(result.target_distance_m, ' m')}",
        f"Cross-track error (aircraft)    : {_fmt(result.aircraft_cross_track_error_m, ' m')}",
        f"Wind             : {_fmt(result.wind_speed_mps, ' m/s')} @ "
        f"{_fmt(result.wind_direction_from_deg, ' deg')} (source={result.wind_source}, quality={result.wind_quality})",
        f"Prediction       : {'STABLE' if result.prediction_stable else 'NOT STABLE'}, confidence={result.confidence}",
        f"Final Release Distance   : {_fmt(result.final_release_distance_m, ' m')}",
        f"Predicted Impact Error   : {_fmt(result.predicted_impact_error_2d_m, ' m')}",
    ]
    if block_reasons:
        lines.append("RELEASE BLOCKED")
        lines.append("REASONS:")
        lines.extend(f"    {r}" for r in block_reasons)
    else:
        lines.append("RELEASE: READY" if not runtime.state_machine.released else "RELEASE: DONE")
    lines.append("")
    return "\n".join(lines) + "\n"


def run_cycle(runtime: PayloadRuntime, telemetry: Dict, mode: str,
              servo: "FakeServoController | MAVLinkServoController",
              logger: DropSystemLogger, empirical_correction_fn=None,
              live_log_path: Optional[str] = None) -> bool:
    """Returns True if a cycle was actually run, False if it was skipped
    because the aircraft's own position/altitude hasn't arrived over
    MAVLink yet this run (LIVE/DRY_RUN only — SIMULATION always has it).
    """
    if "aircraft_lat" not in telemetry or "altitude_m" not in telemetry:
        print(f"PAYLOAD {runtime.payload_id}: waiting for first GLOBAL_POSITION_INT "
              "from the flight controller — no aircraft position/altitude yet.")
        return False

    origin_lat = config.LOCAL_ORIGIN_LAT if config.LOCAL_ORIGIN_LAT is not None else telemetry["aircraft_lat"]
    origin_lon = config.LOCAL_ORIGIN_LON if config.LOCAL_ORIGIN_LON is not None else telemetry["aircraft_lon"]

    altitude_filtered = runtime.altitude_filter.update(telemetry["altitude_m"])
    ground_speed_raw = math.hypot(telemetry["ground_velocity_n"], telemetry["ground_velocity_e"])
    runtime.ground_speed_filter.update(ground_speed_raw)
    ground_track = ground_track_from_velocity(telemetry["ground_velocity_n"], telemetry["ground_velocity_e"])
    runtime.heading_filter.update(telemetry.get("heading_deg", ground_track))

    aircraft_local = latlon_to_local(telemetry["aircraft_lat"], telemetry["aircraft_lon"], origin_lat, origin_lon)
    waypoint_passed = runtime.waypoint_validator.update(
        mission_seq=telemetry.get("mission_seq", 0),
        current_position=aircraft_local,
        waypoint_prev_position=runtime.waypoint_prev_local,
        waypoint_current_position=runtime.waypoint_current_local,
    )

    servo_mapping_valid = False
    try:
        servo_mapping_valid = servo.validate_mapping(runtime.target["servo_channel"])
    except Exception:
        servo_mapping_valid = False

    # Each payload has its own drop-zone geofence — payload 1 and
    # payload 2 are checked against different polygons, not one shared
    # mission-wide fence (that's a separate concept, see
    # krti-flight-software/batas_koordinat.py).
    geofence_status = geofence.check_geofence(
        telemetry["aircraft_lat"], telemetry["aircraft_lon"], runtime.target.get("geofence")
    )

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
        ground_speed_valid=telemetry.get("ground_speed_valid", False), airspeed_valid=telemetry.get("airspeed_valid", True),
        heartbeat_valid=telemetry.get("heartbeat_valid", False),
        telemetry_health=telemetry.get("telemetry_health", "CRITICAL"),
        geofence_inside=geofence_status.inside, geofence_source=geofence_status.source,
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

    # Mirrors the same "is the prediction itself well-formed" check
    # predict_drop_point() applies internally (target valid + release
    # distance finite + altitude valid) — NOT the same thing as
    # result.release_allowed, which already folds in every gate
    # including this one and would make this circular.
    prediction_valid = result.target_valid and math.isfinite(result.final_release_distance_m) and result.altitude_valid

    gates = GateInputs(
        gps_valid=telemetry.get("gps_valid", False), altitude_valid=result.altitude_valid,
        ground_speed_valid=telemetry.get("ground_speed_valid", False), airspeed_valid=telemetry.get("airspeed_valid", True),
        heartbeat_valid=telemetry.get("heartbeat_valid", False), ekf_valid=telemetry.get("ekf_valid", False),
        geofence_valid=geofence_status.inside,
        target_valid=result.target_valid, target_box_valid=result.target_box_valid,
        waypoint_passed=waypoint_passed,
        corridor_valid=abs(result.aircraft_cross_track_error_m) <= runtime.target["max_cross_track_error_m"],
        target_ahead=result.target_distance_m > 0,
        prediction_valid=prediction_valid,
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

    if live_log_path:
        try:
            with open(live_log_path, "a") as f:
                f.write(_format_payload_detail(runtime, result, waypoint_passed, block_reasons))
        except OSError as exc:
            print(f"WARNING: could not write {live_log_path}: {exc}")

    return True


def _make_mapping_check(mavlink: MAVLinkInterface, expected_function: Optional[int]):
    """SERVOx_FUNCTION mapping check backed by a REAL flight-controller
    parameter read. Returns False (mapping unconfirmed -> release
    blocked) whenever the expected function is UNKNOWN
    (config.SERVO_*_EXPECTED_FUNCTION is None) or the param hasn't
    arrived yet — never assumes a channel is correctly wired.
    """
    def check(channel: int) -> bool:
        if expected_function is None:
            return False
        actual = mavlink.get_servo_function_param(channel)
        return actual is not None and actual == expected_function
    return check


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mode", choices=config.OPERATING_MODES, default=config.DEFAULT_OPERATING_MODE)
    parser.add_argument("--cycles", type=int, default=0, help="0 = run forever (Ctrl+C to stop)")
    args = parser.parse_args()

    print(f"Starting drop_system in {args.mode} mode "
          f"(ENABLE_LIVE_RELEASE={config.ENABLE_LIVE_RELEASE})")

    # Create/reset the live status file immediately, before MAVLink
    # connect/mission-fetch or the GPS-fix wait even begin — otherwise
    # `watch cat droping.log` errors with "No such file" for however
    # long that takes.
    try:
        with open(config.LIVE_LOG_FILE, "w") as f:
            f.write(f"drop_system starting in {args.mode} mode — "
                    f"{time.strftime('%Y-%m-%d %H:%M:%S')}\n")
    except OSError as exc:
        print(f"WARNING: could not write {config.LIVE_LOG_FILE}: {exc}")

    # SIMULATION never needs a locked origin (run_cycle's own per-cycle
    # fallback already handles it). DRY_RUN/LIVE need ONE fixed origin
    # for the lifetime of the run — either manually configured, or
    # captured automatically from the aircraft's own position at the
    # first valid GPS fix (fix_type>=3). Once captured it never changes
    # again this run, even if GPS later degrades (that's tracked
    # separately by the normal gps_valid gate).
    origin_locked = args.mode == "SIMULATION" or (
        config.LOCAL_ORIGIN_LAT is not None and config.LOCAL_ORIGIN_LON is not None
    )
    if not origin_locked:
        print("config.LOCAL_ORIGIN_LAT/LON not set — will auto-capture from the "
              "aircraft's own position at the first valid GPS fix. Release stays "
              "on HOLD (waypoint/target/geofence geometry unavailable) until then.")

    runtimes = {pid: PayloadRuntime(pid, target) for pid, target in config.TARGETS.items()}
    logger = DropSystemLogger(config.LOG_CSV_PATH)
    telemetry_health = TelemetryHealth()

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

        print("Fetching mission items from the flight controller...")
        try:
            mission_items = mavlink.fetch_mission_items()
            print(f"Fetched {len(mission_items)} mission items: {sorted(mission_items)}")
        except MAVLinkUnavailableError as exc:
            print(f"WARNING: could not fetch mission items ({exc}); "
                  "waypoint-passed will stay False until this is retried.")
            mission_items = {}
        if origin_locked:
            _resolve_waypoint_positions(runtimes, mission_items, config.LOCAL_ORIGIN_LAT, config.LOCAL_ORIGIN_LON)
        else:
            print("Waypoint positions will be resolved once the local origin is captured.")

        # Each payload's expected SERVOx_FUNCTION is configured
        # independently in config.py (default UNKNOWN -> mapping stays
        # invalid -> release blocked, per spec section 45/158).
        expected_by_channel = {
            t["servo_channel"]: (config.SERVO_1_EXPECTED_FUNCTION if pid == 1 else config.SERVO_2_EXPECTED_FUNCTION)
            for pid, t in config.TARGETS.items()
        }

        def combined_mapping_check(channel: int) -> bool:
            return _make_mapping_check(mavlink, expected_by_channel.get(channel))(channel)

        servo = MAVLinkServoController(mavlink.connection, mapping_check=combined_mapping_check)

    period_s = 1.0 / config.PREDICTION_CYCLE_HZ
    cycle = 0
    try:
        while args.cycles == 0 or cycle < args.cycles:
            t0 = time.monotonic()

            if args.mode == "SIMULATION":
                telemetry = _synthetic_telemetry(t0)
            else:
                # Drain whatever's arrived on the MAVLink socket for
                # this cycle's time budget so _live_telemetry() sees
                # the freshest available messages, not just one.
                deadline = t0 + period_s
                while time.monotonic() < deadline:
                    if mavlink.poll(blocking=False) is None:
                        time.sleep(0.005)
                telemetry = _live_telemetry(mavlink, telemetry_health, now=time.monotonic())

            if not origin_locked:
                captured = _maybe_capture_origin(telemetry)
                if captured is not None:
                    config.LOCAL_ORIGIN_LAT, config.LOCAL_ORIGIN_LON = captured
                    origin_locked = True
                    print(f"LOCAL ORIGIN CAPTURED (source=AUTO_FIRST_GPS_FIX): "
                          f"lat={config.LOCAL_ORIGIN_LAT:.7f}, lon={config.LOCAL_ORIGIN_LON:.7f}")
                    _resolve_waypoint_positions(runtimes, mission_items, config.LOCAL_ORIGIN_LAT, config.LOCAL_ORIGIN_LON)
                else:
                    print(f"\r[{time.strftime('%H:%M:%S')}] cycle={cycle} HOLD: waiting for a "
                          "valid GPS fix to establish local origin — release blocked."
                          "          ", end="", flush=True)
                    try:
                        with open(config.LIVE_LOG_FILE, "w") as f:
                            f.write(
                                f"drop_system live status — {time.strftime('%Y-%m-%d %H:%M:%S')} "
                                f"— cycle {cycle} — mode={args.mode}\n\n"
                                "HOLD: waiting for a valid GPS fix to establish the local "
                                "origin (config.LOCAL_ORIGIN_LAT/LON auto-capture) — no "
                                "aircraft/target geometry is computable yet, both payloads "
                                "blocked.\n"
                                f"gps_valid={telemetry.get('gps_valid')}  "
                                f"aircraft_lat={telemetry.get('aircraft_lat', 'N/A')}  "
                                f"aircraft_lon={telemetry.get('aircraft_lon', 'N/A')}\n"
                            )
                    except OSError as exc:
                        print(f"WARNING: could not write {config.LIVE_LOG_FILE}: {exc}")
                    cycle += 1
                    elapsed = time.monotonic() - t0
                    time.sleep(max(0.0, period_s - elapsed))
                    continue

            # Fresh snapshot each cycle: truncate + header, then each
            # run_cycle() call below appends its payload's block. Full
            # numbers live here, not on the console (see module
            # docstring / config.LIVE_LOG_FILE).
            try:
                with open(config.LIVE_LOG_FILE, "w") as f:
                    f.write(f"drop_system live status — {time.strftime('%Y-%m-%d %H:%M:%S')} "
                            f"— cycle {cycle} — mode={args.mode}\n\n")
            except OSError as exc:
                print(f"WARNING: could not write {config.LIVE_LOG_FILE}: {exc}")

            for pid, runtime in runtimes.items():
                run_cycle(runtime, telemetry, args.mode, servo, logger,
                          live_log_path=config.LIVE_LOG_FILE)

            status = " ".join(
                f"P{pid}:{'DONE' if rt.state_machine.released else ('READY' if not rt.state_machine.last_block_reasons else f'HOLD({len(rt.state_machine.last_block_reasons)})')}"
                for pid, rt in runtimes.items()
            )
            print(f"\r[{time.strftime('%H:%M:%S')}] cycle={cycle} OK  telemetry={telemetry.get('telemetry_health', 'n/a')}  "
                  f"{status}  -> {config.LIVE_LOG_FILE}          ", end="", flush=True)

            cycle += 1
            elapsed = time.monotonic() - t0
            time.sleep(max(0.0, period_s - elapsed))
    except KeyboardInterrupt:
        print("\nStopped (Ctrl+C)")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
