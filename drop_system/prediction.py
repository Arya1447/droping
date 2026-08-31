"""Core drop-point prediction pipeline (spec sections 1, 100-102, 164).

Central question: "at what position should the aircraft command release
so the payload is predicted to land as close as possible to the
target?" `predict_drop_point()` answers that for one payload/cycle,
returning every quantity the spec requires kept distinct (flight time,
drop distance, servo displacement, target distance, release distance,
impact point, impact error) plus the full release-gate decision.

This module contains the ONE physics core (`compute_physics_core`) used
both live (this file) and offline by monte_carlo.py, so the two never
drift apart into two different models.
"""

from __future__ import annotations

import math
import random
from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional, Tuple

import config
import physics_model
import wind_model
from coordinate_utils import (
    along_cross_track, ground_track_from_velocity, horizontal_to_ne,
    latlon_to_local, local_to_latlon,
)
from state_machine import GateInputs, evaluate_gates
from target_validator import TargetBox, is_inside_target_box, target_ahead, target_valid


# ============================================================
# Physics core — shared by live prediction and Monte Carlo
# ============================================================

@dataclass
class PhysicsCoreInputs:
    payload_mass_kg: float
    altitude_m: float
    ground_velocity_n: float
    ground_velocity_e: float
    ground_velocity_u: float  # +up
    servo_delay_s: float
    g: float = config.GRAVITY_MPS2
    drag_enabled: bool = False
    cd: Optional[float] = None
    area_m2: Optional[float] = None
    rho: Optional[float] = None
    wind_n: float = 0.0
    wind_e: float = 0.0
    wind_u: float = 0.0


@dataclass
class PhysicsCoreResult:
    t_flight_s: float
    drop_distance_m: float
    drop_north_m: float
    drop_east_m: float
    servo_displacement_m: float
    servo_north_m: float
    servo_east_m: float
    gravity_force_n: float
    gravity_acceleration_mps2: float
    drag_force_n: Optional[float]
    drag_acceleration_mps2: Optional[float]
    ballistic_coefficient_kgm2: Optional[float]


def compute_physics_core(inputs: PhysicsCoreInputs) -> PhysicsCoreResult:
    """Pure function: ground-velocity + altitude + delay -> flight
    dynamics. Ballistic model by default; drag model only if
    drag_enabled and cd/area/rho are all supplied (never fabricated).
    """
    servo_n = inputs.ground_velocity_n * inputs.servo_delay_s
    servo_e = inputs.ground_velocity_e * inputs.servo_delay_s
    servo_disp = math.hypot(servo_n, servo_e)

    gravity_force = physics_model.gravity_force_n(inputs.payload_mass_kg, inputs.g)
    gravity_accel = physics_model.gravity_acceleration(inputs.g)

    if inputs.drag_enabled and inputs.cd is not None and inputs.area_m2 is not None and inputs.rho is not None:
        traj = physics_model.integrate_trajectory_with_drag(
            mass_kg=inputs.payload_mass_kg,
            rho=inputs.rho, cd=inputs.cd, area_m2=inputs.area_m2,
            v0_n=inputs.ground_velocity_n, v0_e=inputs.ground_velocity_e, v0_u=inputs.ground_velocity_u,
            wind_n=inputs.wind_n, wind_e=inputs.wind_e, wind_u=inputs.wind_u,
            height_m=inputs.altitude_m, g=inputs.g,
        )
        t_flight = traj["t_flight"]
        drop_n, drop_e = traj["north"], traj["east"]
        drop_dist = traj["drop_distance"]

        v_rel_n = inputs.ground_velocity_n - inputs.wind_n
        v_rel_e = inputs.ground_velocity_e - inputs.wind_e
        v_rel_u = inputs.ground_velocity_u - inputs.wind_u
        a_n, a_e, a_u = physics_model.drag_acceleration(
            inputs.rho, inputs.cd, inputs.area_m2, inputs.payload_mass_kg, v_rel_n, v_rel_e, v_rel_u
        )
        drag_accel = math.sqrt(a_n ** 2 + a_e ** 2 + a_u ** 2)
        drag_force = drag_accel * inputs.payload_mass_kg
        beta = physics_model.ballistic_coefficient(inputs.payload_mass_kg, inputs.cd, inputs.area_m2)
    else:
        t_flight = physics_model.time_of_flight(inputs.altitude_m, inputs.ground_velocity_u, inputs.g)
        drop_n = inputs.ground_velocity_n * t_flight
        drop_e = inputs.ground_velocity_e * t_flight
        drop_dist = math.hypot(drop_n, drop_e)
        drag_force = None
        drag_accel = None
        beta = None

    return PhysicsCoreResult(
        t_flight_s=t_flight,
        drop_distance_m=drop_dist,
        drop_north_m=drop_n,
        drop_east_m=drop_e,
        servo_displacement_m=servo_disp,
        servo_north_m=servo_n,
        servo_east_m=servo_e,
        gravity_force_n=gravity_force,
        gravity_acceleration_mps2=gravity_accel,
        drag_force_n=drag_force,
        drag_acceleration_mps2=drag_accel,
        ballistic_coefficient_kgm2=beta,
    )


# ============================================================
# Prediction stability tracking (spec section 42/92)
# ============================================================

class PredictionStabilityTracker:
    def __init__(self, stable_cycles: int = config.PREDICTION_STABLE_CYCLES,
                 max_change_m: float = config.MAX_PREDICTION_CHANGE_M):
        self.stable_cycles = stable_cycles
        self.max_change_m = max_change_m
        self._history: List[float] = []

    def update(self, predicted_release_distance_m: float) -> bool:
        self._history.append(predicted_release_distance_m)
        if len(self._history) > self.stable_cycles:
            self._history = self._history[-self.stable_cycles:]
        if len(self._history) < self.stable_cycles:
            return False
        deltas = [abs(self._history[i] - self._history[i - 1]) for i in range(1, len(self._history))]
        return all(d <= self.max_change_m for d in deltas)

    def reset(self) -> None:
        self._history.clear()


# ============================================================
# Full prediction result
# ============================================================

@dataclass
class PredictionResult:
    payload_id: int
    payload_mass_kg: float
    payload_mass_gram: float
    mass_source: str
    mass_uncertainty_kg: Optional[float]

    altitude_raw_m: float
    altitude_filtered_m: float
    altitude_source: str
    altitude_valid: bool

    ground_speed_mps: float
    ground_track_deg: float
    air_speed_mps: Optional[float]
    heading_deg: Optional[float]
    flight_path_angle_deg: Optional[float]

    wind_speed_mps: Optional[float]
    wind_direction_from_deg: Optional[float]
    wind_source: str
    wind_quality: str

    predicted_fall_time_s: float
    predicted_drop_distance_m: float
    servo_displacement_m: float

    target_distance_m: float
    predicted_release_distance_m: float
    wind_correction_m: float
    empirical_correction_m: float
    final_release_distance_m: float

    predicted_impact_local_n: float
    predicted_impact_local_e: float
    predicted_impact_lat: Optional[float]
    predicted_impact_lon: Optional[float]

    signed_impact_error_along_m: float
    impact_error_cross_m: float
    predicted_impact_error_2d_m: float
    impact_status: str  # OVERSHOOT / UNDERSHOOT / ON_TARGET

    aircraft_cross_track_error_m: float

    gravity_force_n: float
    gravity_acceleration_mps2: float
    drag_force_n: Optional[float]
    drag_acceleration_mps2: Optional[float]
    ballistic_coefficient_kgm2: Optional[float]

    target_valid: bool
    target_box_valid: bool
    predicted_impact_inside_box: bool
    waypoint_valid: bool
    telemetry_health: str

    prediction_stable: bool
    prediction_uncertainty_m: Optional[float]  # std of live-sampled impact_error_2d
    confidence: str  # HIGH / MEDIUM / LOW / UNKNOWN

    release_allowed: bool
    release_block_reason: List[str]


def _empirical_correction(fn: Optional[Callable[..., float]], **kwargs) -> Tuple[float, str]:
    if fn is None:
        return 0.0, "NONE"
    try:
        return float(fn(**kwargs)), "CALIBRATED"
    except Exception:
        return 0.0, "CALIBRATION_ERROR"


def _live_uncertainty_sample(
    core_inputs: PhysicsCoreInputs,
    ground_speed_sigma: Optional[float],
    heading_sigma_deg: Optional[float],
    altitude_sigma_m: Optional[float],
    ground_track_deg: float,
    n_samples: int = 200,
) -> Optional[float]:
    """Lightweight (n=200, not the offline 10,000-trial Monte Carlo)
    resampling of the ballistic core to get a live impact-error spread
    estimate. Cheap enough to run every prediction cycle (spec section
    117 only forbids the full offline MC/training/cross-validation
    workload on the live loop, not a small closed-form resample).
    Returns None if no sigma is available for perturbation.
    """
    if ground_speed_sigma is None and heading_sigma_deg is None and altitude_sigma_m is None:
        return None

    ground_speed = math.hypot(core_inputs.ground_velocity_n, core_inputs.ground_velocity_e)
    drop_norths = []
    for _ in range(n_samples):
        gs = random.gauss(ground_speed, ground_speed_sigma) if ground_speed_sigma else ground_speed
        hdg = random.gauss(ground_track_deg, heading_sigma_deg) if heading_sigma_deg else ground_track_deg
        alt = max(0.1, random.gauss(core_inputs.altitude_m, altitude_sigma_m) if altitude_sigma_m else core_inputs.altitude_m)
        vn, ve = horizontal_to_ne(gs, hdg)
        sample_inputs = PhysicsCoreInputs(
            payload_mass_kg=core_inputs.payload_mass_kg,
            altitude_m=alt,
            ground_velocity_n=vn, ground_velocity_e=ve, ground_velocity_u=core_inputs.ground_velocity_u,
            servo_delay_s=core_inputs.servo_delay_s, g=core_inputs.g,
            drag_enabled=core_inputs.drag_enabled, cd=core_inputs.cd,
            area_m2=core_inputs.area_m2, rho=core_inputs.rho,
            wind_n=core_inputs.wind_n, wind_e=core_inputs.wind_e, wind_u=core_inputs.wind_u,
        )
        result = compute_physics_core(sample_inputs)
        total_n = result.servo_north_m + result.drop_north_m
        drop_norths.append(math.hypot(total_n, result.servo_east_m + result.drop_east_m))

    mean = sum(drop_norths) / len(drop_norths)
    var = sum((d - mean) ** 2 for d in drop_norths) / (len(drop_norths) - 1)
    return math.sqrt(var)


def predict_drop_point(
    *,
    payload_id: int,
    payload_mass_kg: float,
    mass_source: str,
    mass_uncertainty_kg: Optional[float],

    aircraft_lat: float,
    aircraft_lon: float,
    origin_lat: float,
    origin_lon: float,

    target: dict,  # config.TARGET_1 / TARGET_2 shape

    altitude_raw_m: float,
    altitude_filtered_m: float,
    altitude_source: str,
    altitude_valid: bool,

    ground_velocity_n: Optional[float],
    ground_velocity_e: Optional[float],
    ground_velocity_u: Optional[float],
    ground_speed_mps_fallback: Optional[float] = None,
    ground_track_deg_fallback: Optional[float] = None,

    air_speed_mps: Optional[float] = None,
    heading_deg: Optional[float] = None,
    flight_path_angle_deg: Optional[float] = None,

    wind_speed_mps: Optional[float] = None,
    wind_direction_from_deg: Optional[float] = None,
    wind_source: str = "UNKNOWN",
    wind_quality: str = "UNKNOWN",

    servo_delay_s: float = config.SERVO_DELAY_S_TEST,
    servo_delay_sigma_s: Optional[float] = None,

    waypoint_passed: bool = False,
    gps_valid: bool = False,
    ekf_valid: bool = False,
    ground_speed_valid: bool = False,
    airspeed_valid: bool = False,
    heartbeat_valid: bool = False,
    telemetry_health: str = "CRITICAL",

    servo_mapping_valid: bool = False,
    servo_safety_valid: bool = False,
    live_release_enabled: bool = config.ENABLE_LIVE_RELEASE,
    already_released: bool = False,

    drag_enabled: bool = config.DRAG_ENABLED,
    cd: Optional[float] = config.DRAG_CD,
    area_m2: Optional[float] = config.DRAG_REFERENCE_AREA_M2,
    rho: Optional[float] = config.AIR_DENSITY_KGM3,

    empirical_correction_fn: Optional[Callable[..., float]] = None,

    stability_tracker: Optional[PredictionStabilityTracker] = None,
    ground_speed_sigma_mps: Optional[float] = None,
    heading_sigma_deg: Optional[float] = None,
    altitude_sigma_m: Optional[float] = None,
) -> PredictionResult:
    """Compute the full drop-point prediction for one payload, one cycle.
    See module docstring / spec sections 1, 100-102, 164 for the field
    contract. Every gate default is fail-closed (False) so an unwired
    caller blocks release rather than silently allowing it.
    """

    # --- ground velocity vector: prefer direct filtered vector; else
    # construct from ground_speed + explicit ground_track; heading is
    # NEVER silently substituted for ground track (spec section 8/154).
    if ground_velocity_n is not None and ground_velocity_e is not None:
        v_n, v_e = ground_velocity_n, ground_velocity_e
        ground_track = ground_track_from_velocity(v_n, v_e)
    elif ground_speed_mps_fallback is not None and ground_track_deg_fallback is not None:
        v_n, v_e = horizontal_to_ne(ground_speed_mps_fallback, ground_track_deg_fallback)
        ground_track = ground_track_deg_fallback
    else:
        raise ValueError(
            "insufficient ground-velocity data: need either "
            "(ground_velocity_n, ground_velocity_e) or "
            "(ground_speed_mps_fallback, ground_track_deg_fallback); "
            "heading is not a valid substitute for ground track"
        )
    ground_speed = math.hypot(v_n, v_e)

    if ground_velocity_u is not None:
        v_u = ground_velocity_u
    elif flight_path_angle_deg is not None:
        v_u = ground_speed * math.tan(math.radians(flight_path_angle_deg))
    else:
        v_u = 0.0  # ASSUMED level flight — no vertical velocity data available

    # --- wind vector (only needed if drag active) ---
    wind_n = wind_e = wind_u = 0.0
    if wind_speed_mps is not None and wind_direction_from_deg is not None and not math.isnan(wind_direction_from_deg):
        wind_n, wind_e = wind_model.wind_vector_from_speed_direction(wind_speed_mps, wind_direction_from_deg)

    # --- physics core ---
    core_inputs = PhysicsCoreInputs(
        payload_mass_kg=payload_mass_kg,
        altitude_m=altitude_filtered_m,
        ground_velocity_n=v_n, ground_velocity_e=v_e, ground_velocity_u=v_u,
        servo_delay_s=servo_delay_s,
        drag_enabled=drag_enabled, cd=cd, area_m2=area_m2, rho=rho,
        wind_n=wind_n, wind_e=wind_e, wind_u=wind_u,
    )
    core = compute_physics_core(core_inputs)

    # --- geometry: aircraft/target in local NEU ---
    aircraft_n, aircraft_e = latlon_to_local(aircraft_lat, aircraft_lon, origin_lat, origin_lon)
    t_valid = target_valid(target.get("lat"), target.get("lon"))
    if t_valid:
        target_n, target_e = latlon_to_local(target["lat"], target["lon"], origin_lat, origin_lon)
        delta_ac_target_n = target_n - aircraft_n
        delta_ac_target_e = target_e - aircraft_e
        target_along, target_cross = along_cross_track(delta_ac_target_n, delta_ac_target_e, ground_track)
    else:
        target_n = target_e = 0.0
        target_along, target_cross = 0.0, 0.0

    target_distance = target_along  # spec section D: along-track distance to target

    predicted_release_distance = target_distance - core.servo_displacement_m - core.drop_distance_m

    wind_correction = 0.0  # ballistic-only model: wind has no effect on payload trajectory
    # once released (no drag => no wind coupling); only nonzero if a
    # drag-based wind-sensitivity term is derived from the drag model
    # itself, which this baseline does not add on top of the already
    # wind-aware drag integrator above (avoids double-counting).

    empirical_correction, correction_source = _empirical_correction(
        empirical_correction_fn,
        payload_mass_kg=payload_mass_kg, ground_speed_mps=ground_speed,
        air_speed_mps=air_speed_mps, altitude_m=altitude_filtered_m,
        wind_speed_mps=wind_speed_mps, wind_direction_deg=wind_direction_from_deg,
        heading_deg=heading_deg, flight_path_angle_deg=flight_path_angle_deg,
        servo_delay_s=servo_delay_s,
    )

    final_release_distance = predicted_release_distance + wind_correction + empirical_correction

    # --- predicted impact point ---
    release_n = aircraft_n + core.servo_north_m
    release_e = aircraft_e + core.servo_east_m
    impact_n = release_n + core.drop_north_m
    impact_e = release_e + core.drop_east_m

    if origin_lat is not None and origin_lon is not None:
        impact_lat, impact_lon = local_to_latlon(impact_n, impact_e, origin_lat, origin_lon)
    else:
        impact_lat = impact_lon = None

    if t_valid:
        e_along, e_cross = along_cross_track(impact_n - target_n, impact_e - target_e, ground_track)
    else:
        e_along, e_cross = float("nan"), float("nan")
    e_2d = math.hypot(e_along, e_cross) if t_valid else float("nan")
    if not t_valid:
        impact_status = "UNKNOWN"
    elif e_along > 0.5:
        impact_status = "OVERSHOOT"
    elif e_along < -0.5:
        impact_status = "UNDERSHOOT"
    else:
        impact_status = "ON_TARGET"

    box = TargetBox(**target["box"]) if t_valid else None
    impact_inside_box = (
        is_inside_target_box(impact_n - target_n, impact_e - target_e, box)
        if (t_valid and box is not None) else False
    )

    corridor_ok = t_valid and abs(target_cross) <= target.get("max_cross_track_error_m", float("inf"))
    ahead_ok = t_valid and target_ahead(target_along)

    # --- stability ---
    if stability_tracker is not None:
        stable = stability_tracker.update(final_release_distance)
    else:
        stable = False

    # --- light live uncertainty resample ---
    unc = _live_uncertainty_sample(
        core_inputs, ground_speed_sigma_mps, heading_sigma_deg, altitude_sigma_m, ground_track
    )

    prediction_valid = t_valid and math.isfinite(final_release_distance) and altitude_valid

    gates = GateInputs(
        gps_valid=gps_valid,
        altitude_valid=altitude_valid,
        ground_speed_valid=ground_speed_valid,
        airspeed_valid=airspeed_valid,
        heartbeat_valid=heartbeat_valid,
        ekf_valid=ekf_valid,
        target_valid=t_valid,
        target_box_valid=t_valid,
        waypoint_passed=waypoint_passed,
        corridor_valid=corridor_ok,
        target_ahead=ahead_ok,
        prediction_valid=prediction_valid,
        predicted_impact_inside_box=impact_inside_box,
        release_distance_valid=math.isfinite(final_release_distance),
        release_window_valid=ahead_ok and prediction_valid,
        prediction_stable=stable,
        servo_mapping_valid=servo_mapping_valid,
        servo_safety_valid=servo_safety_valid,
        live_release_enabled=live_release_enabled,
    )
    block_reasons = evaluate_gates(gates, already_released)
    release_allowed = len(block_reasons) == 0

    if telemetry_health != "HEALTHY":
        confidence = "LOW"
    elif mass_source in ("MEASURED", "CONFIGURED") and stable and unc is not None and unc <= config.MAX_ACCEPTABLE_IMPACT_ERROR_M:
        confidence = "HIGH"
    elif stable:
        confidence = "MEDIUM"
    else:
        confidence = "LOW"

    return PredictionResult(
        payload_id=payload_id,
        payload_mass_kg=payload_mass_kg,
        payload_mass_gram=payload_mass_kg * 1000.0,
        mass_source=mass_source,
        mass_uncertainty_kg=mass_uncertainty_kg,

        altitude_raw_m=altitude_raw_m,
        altitude_filtered_m=altitude_filtered_m,
        altitude_source=altitude_source,
        altitude_valid=altitude_valid,

        ground_speed_mps=ground_speed,
        ground_track_deg=ground_track,
        air_speed_mps=air_speed_mps,
        heading_deg=heading_deg,
        flight_path_angle_deg=flight_path_angle_deg,

        wind_speed_mps=wind_speed_mps,
        wind_direction_from_deg=wind_direction_from_deg,
        wind_source=wind_source,
        wind_quality=wind_quality,

        predicted_fall_time_s=core.t_flight_s,
        predicted_drop_distance_m=core.drop_distance_m,
        servo_displacement_m=core.servo_displacement_m,

        target_distance_m=target_distance,
        predicted_release_distance_m=predicted_release_distance,
        wind_correction_m=wind_correction,
        empirical_correction_m=empirical_correction,
        final_release_distance_m=final_release_distance,

        predicted_impact_local_n=impact_n,
        predicted_impact_local_e=impact_e,
        predicted_impact_lat=impact_lat,
        predicted_impact_lon=impact_lon,

        signed_impact_error_along_m=e_along,
        impact_error_cross_m=e_cross,
        predicted_impact_error_2d_m=e_2d,
        impact_status=impact_status,

        aircraft_cross_track_error_m=target_cross,

        gravity_force_n=core.gravity_force_n,
        gravity_acceleration_mps2=core.gravity_acceleration_mps2,
        drag_force_n=core.drag_force_n,
        drag_acceleration_mps2=core.drag_acceleration_mps2,
        ballistic_coefficient_kgm2=core.ballistic_coefficient_kgm2,

        target_valid=t_valid,
        target_box_valid=t_valid,
        predicted_impact_inside_box=impact_inside_box,
        waypoint_valid=waypoint_passed,
        telemetry_health=telemetry_health,

        prediction_stable=stable,
        prediction_uncertainty_m=unc,
        confidence=confidence,

        release_allowed=release_allowed,
        release_block_reason=block_reasons,
    )
