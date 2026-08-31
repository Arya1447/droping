"""Offline Monte Carlo sensitivity/accuracy analysis (spec sections 3,
80-83, 133, 144).

Design (spec sections 63/64/80: never randomize target or aircraft GPS
position/target distance by default):

  1. A single NOMINAL physics core run (TEST_CASE mean values, no
     sampling) fixes the release-trigger along-track distance
     `R_release_cmd_nominal = servo_displacement_nominal + drop_distance_nominal`.
     This is the along-track distance-to-target at which release fires
     for every trial — a fixed geometric trigger, not a random variable.
  2. Each trial samples the UNCERTAIN inputs only (altitude, ground
     speed, heading, flight-path angle, wind, servo delay, and payload
     mass only because config.TEST_UNCERTAINTY documents an explicit
     ASSUMED sigma for it) and recomputes the physics core with those
     sampled values.
  3. The resulting along-track/cross-track displacement is compared
     against the fixed trigger distance to get this trial's impact
     error — i.e. "given the aircraft always releases at the nominal
     computed distance, how much does real-world variability in flight
     conditions move the actual impact point?"

Distribution choice (spec section 81): Normal(mean, sigma) is used for
every sampled input. This is the DEFAULT because config.TEST_UNCERTAINTY
provides only mean+sigma (no empirical dataset shape to justify a
different distribution) — if/when real flight-test residual data is
collected, calibration.py's outlier/residual analysis can inform
switching specific inputs to an empirical or bounded distribution
instead. This is not claimed to be more accurate than Normal today.
"""

from __future__ import annotations

import math
import random
from dataclasses import dataclass
from typing import Dict, List, Optional

import config
from coordinate_utils import along_cross_track, horizontal_to_ne
from prediction import PhysicsCoreInputs, compute_physics_core
from uncertainty import summarize_distribution


@dataclass
class MonteCarloTrial:
    trial: int
    altitude_m: float
    ground_speed_mps: float
    air_speed_mps: float
    heading_deg: float
    ground_track_deg: float
    flight_path_angle_deg: float
    wind_speed_mps: float
    wind_direction_deg: float
    payload_mass_kg: float
    servo_delay_s: float
    flight_time_s: float
    drop_distance_m: float
    servo_displacement_m: float
    target_distance_m: float
    required_release_distance_m: float
    predicted_impact_along_m: float
    predicted_impact_cross_m: float
    signed_impact_error_m: float
    absolute_impact_error_m: float
    impact_status: str


def _nominal_release_trigger(test_case: config.TestCase) -> float:
    v_n, v_e = horizontal_to_ne(test_case.ground_speed_mps, test_case.heading_deg)
    v_u = test_case.ground_speed_mps * math.tan(math.radians(test_case.flight_path_angle_deg))
    core = compute_physics_core(PhysicsCoreInputs(
        payload_mass_kg=test_case.payload_mass_kg,
        altitude_m=test_case.altitude_m,
        ground_velocity_n=v_n, ground_velocity_e=v_e, ground_velocity_u=v_u,
        servo_delay_s=test_case.servo_delay_s,
    ))
    return core.servo_displacement_m + core.drop_distance_m


def run_monte_carlo(
    test_case: config.TestCase = config.TEST_CASE,
    uncertainty: config.TestUncertainty = config.TEST_UNCERTAINTY,
    n_trials: Optional[int] = None,
    seed: Optional[int] = None,
) -> List[MonteCarloTrial]:
    n_trials = n_trials or test_case.monte_carlo_trials
    rng = random.Random(seed)

    r_release_cmd = _nominal_release_trigger(test_case)
    target_bearing_deg = test_case.heading_deg  # ASSUMED: target lies along nominal track

    trials: List[MonteCarloTrial] = []
    for i in range(n_trials):
        altitude = max(0.1, rng.gauss(test_case.altitude_m, uncertainty.altitude_sigma_m))
        ground_speed = max(0.1, rng.gauss(test_case.ground_speed_mps, uncertainty.ground_speed_sigma_mps))
        air_speed = max(0.1, rng.gauss(test_case.air_speed_mps, uncertainty.air_speed_sigma_mps))
        heading = rng.gauss(test_case.heading_deg, uncertainty.heading_sigma_deg)
        fpa = rng.gauss(test_case.flight_path_angle_deg, uncertainty.flight_path_angle_sigma_deg)
        wind_speed = max(0.0, rng.gauss(test_case.wind_speed_mps, uncertainty.wind_speed_sigma_mps))
        wind_dir = rng.gauss(180.0, uncertainty.wind_direction_sigma_deg)  # ASSUMED nominal wind FROM south for test
        mass = max(0.01, rng.gauss(test_case.payload_mass_kg, uncertainty.payload_mass_sigma_kg))
        servo_delay = max(0.0, rng.gauss(test_case.servo_delay_s, uncertainty.servo_delay_sigma_s))

        v_n, v_e = horizontal_to_ne(ground_speed, heading)
        v_u = ground_speed * math.tan(math.radians(fpa))

        core = compute_physics_core(PhysicsCoreInputs(
            payload_mass_kg=mass, altitude_m=altitude,
            ground_velocity_n=v_n, ground_velocity_e=v_e, ground_velocity_u=v_u,
            servo_delay_s=servo_delay,
        ))

        total_n = core.servo_north_m + core.drop_north_m
        total_e = core.servo_east_m + core.drop_east_m
        along, cross = along_cross_track(total_n, total_e, target_bearing_deg)

        e_along = along - r_release_cmd
        e_cross = cross
        e_abs = math.hypot(e_along, e_cross)

        if e_along > 0.5:
            status = "OVERSHOOT"
        elif e_along < -0.5:
            status = "UNDERSHOOT"
        else:
            status = "ON_TARGET"

        trials.append(MonteCarloTrial(
            trial=i, altitude_m=altitude, ground_speed_mps=ground_speed,
            air_speed_mps=air_speed, heading_deg=heading, ground_track_deg=heading,
            flight_path_angle_deg=fpa, wind_speed_mps=wind_speed, wind_direction_deg=wind_dir,
            payload_mass_kg=mass, servo_delay_s=servo_delay,
            flight_time_s=core.t_flight_s, drop_distance_m=core.drop_distance_m,
            servo_displacement_m=core.servo_displacement_m,
            target_distance_m=r_release_cmd, required_release_distance_m=along,
            predicted_impact_along_m=along, predicted_impact_cross_m=cross,
            signed_impact_error_m=e_along, absolute_impact_error_m=e_abs,
            impact_status=status,
        ))

    return trials


def compute_performance_metrics(trials: List[MonteCarloTrial],
                                 max_acceptable_m: float = config.MAX_ACCEPTABLE_IMPACT_ERROR_M) -> Dict:
    """spec sections 83, 133, 144: MAE/RMSE/MedianAE/STD/P95/Max plus
    within/outside-5m percentages, computed on impact_error_2d
    (absolute_impact_error_m).
    """
    abs_errors = [t.absolute_impact_error_m for t in trials]
    signed_errors = [t.signed_impact_error_m for t in trials]
    n = len(abs_errors)

    dist = summarize_distribution(abs_errors)
    mae = dist.mean
    rmse = math.sqrt(sum(e ** 2 for e in abs_errors) / n)
    within = sum(1 for e in abs_errors if e <= max_acceptable_m)
    within_pct = 100.0 * within / n
    outside_pct = 100.0 - within_pct

    release_dist = summarize_distribution([t.required_release_distance_m for t in trials])
    flight_time_dist = summarize_distribution([t.flight_time_s for t in trials])
    drop_dist_dist = summarize_distribution([t.drop_distance_m for t in trials])

    return {
        "n_trials": n,
        "mean_signed_error_m": sum(signed_errors) / n,
        "mae_m": mae,
        "rmse_m": rmse,
        "median_ae_m": dist.median,
        "std_m": dist.std,
        "p95_abs_error_m": dist.p95,
        "max_abs_error_m": max(abs_errors),
        "within_5m_pct": within_pct,
        "outside_5m_pct": outside_pct,
        "probability_within_5m": within / n,
        "probability_outside_5m": 1.0 - within / n,
        "release_distance_stats": release_dist,
        "flight_time_stats": flight_time_dist,
        "drop_distance_stats": drop_dist_dist,
    }


def evaluate_acceptance(metrics: Dict, policy: Dict = config.ACCEPTANCE_POLICY) -> str:
    """PASS / FAIL / INSUFFICIENT_DATA, per a policy fixed BEFORE seeing
    results (spec section 136). Never chooses thresholds post-hoc.
    """
    if metrics["n_trials"] < policy["min_samples_for_verdict"]:
        return "INSUFFICIENT_DATA"

    mae_ok = metrics["mae_m"] <= policy["require_mae_le"]
    p95_ok = metrics["p95_abs_error_m"] <= policy["require_p95_le"]
    within_ok = metrics["within_5m_pct"] >= policy["require_within_5m_pct_ge"]

    return "PASS" if (mae_ok and p95_ok and within_ok) else "FAIL"
