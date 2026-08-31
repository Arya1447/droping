#!/usr/bin/env python3
"""Offline analysis + visualization (spec sections 110-113, 124,
132-142). Produces the required plot set from Monte Carlo trial data
and (optionally) real flight-test data, plus a PASS/FAIL/INSUFFICIENT_DATA
acceptance report against the <=5 m performance requirement.

Every plot that needs data this run doesn't have (e.g. real GPS target
coordinates, actual post-flight impact measurements) is skipped with an
explicit printed warning rather than being faked.
"""

from __future__ import annotations

import argparse
import os
from typing import List, Optional

import config
from monte_carlo import MonteCarloTrial, compute_performance_metrics, evaluate_acceptance, run_monte_carlo
from target_validator import TargetBox


def _mpl():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    return plt


def plot_release_distance_vs_trial(trials: List[MonteCarloTrial], out_dir: str) -> str:
    plt = _mpl()
    fig, ax = plt.subplots()
    ax.plot([t.trial for t in trials], [t.required_release_distance_m for t in trials], ".", ms=2)
    ax.set_xlabel("Trial")
    ax.set_ylabel("Required Release Distance [m]")
    ax.set_title("Required Release Distance vs Trial\n(distance-to-target at which release should fire — NOT impact error)")
    path = os.path.join(out_dir, "01_release_distance_vs_trial.png")
    fig.savefig(path, dpi=120, bbox_inches="tight")
    plt.close(fig)
    return path


def plot_hist(values, title: str, xlabel: str, filename: str, out_dir: str) -> str:
    plt = _mpl()
    fig, ax = plt.subplots()
    ax.hist(values, bins=40)
    ax.set_xlabel(xlabel)
    ax.set_ylabel("Count")
    ax.set_title(title)
    path = os.path.join(out_dir, filename)
    fig.savefig(path, dpi=120, bbox_inches="tight")
    plt.close(fig)
    return path


def plot_scatter_error_vs(trials: List[MonteCarloTrial], attr: str, xlabel: str, filename: str, out_dir: str) -> str:
    plt = _mpl()
    fig, ax = plt.subplots()
    x = [getattr(t, attr) for t in trials]
    y = [t.absolute_impact_error_m for t in trials]
    ax.scatter(x, y, s=4, alpha=0.4)
    ax.axhline(config.MAX_ACCEPTABLE_IMPACT_ERROR_M, color="r", linestyle="--", label=f"{config.MAX_ACCEPTABLE_IMPACT_ERROR_M} m requirement")
    ax.set_xlabel(xlabel)
    ax.set_ylabel("Absolute Impact Error (2D) [m]")
    ax.set_title(f"Impact Error vs {xlabel}")
    ax.legend()
    path = os.path.join(out_dir, filename)
    fig.savefig(path, dpi=120, bbox_inches="tight")
    plt.close(fig)
    return path


def plot_target_box_with_impacts(trials: List[MonteCarloTrial], box: TargetBox, payload_id: int, out_dir: str) -> str:
    plt = _mpl()
    fig, ax = plt.subplots()
    along = [t.predicted_impact_along_m - t.target_distance_m for t in trials]
    cross = [t.predicted_impact_cross_m for t in trials]
    ax.scatter(cross, along, s=4, alpha=0.4, label="predicted impacts (relative to target)")
    ax.add_patch(plt.Rectangle((-box.west_m, -box.south_m), box.west_m + box.east_m,
                                box.south_m + box.north_m, fill=False, edgecolor="g", label="target box"))
    ax.axhline(0, color="k", linewidth=0.5)
    ax.axvline(0, color="k", linewidth=0.5)
    ax.set_xlabel("Cross-track offset from target [m]")
    ax.set_ylabel("Along-track offset from target [m]")
    ax.set_title(f"Target {payload_id} Box + Predicted Impact Points")
    ax.legend()
    ax.set_aspect("equal", adjustable="datalim")
    path = os.path.join(out_dir, f"{18 + payload_id - 1:02d}_target_{payload_id}_box_impacts.png")
    fig.savefig(path, dpi=120, bbox_inches="tight")
    plt.close(fig)
    return path


def plot_sensitivity_mass(out_dir: str) -> str:
    """Mass sensitivity (spec section 17A-I): 0.45/0.50/0.55 kg are
    MATHEMATICAL test points to illustrate the mass-drag relationship,
    NOT a claim that the actual payload has these three masses.
    """
    import math
    from coordinate_utils import horizontal_to_ne
    from prediction import PhysicsCoreInputs, compute_physics_core

    masses = [0.45, 0.50, 0.55]
    tc = config.TEST_CASE
    v_n, v_e = horizontal_to_ne(tc.ground_speed_mps, tc.heading_deg)

    rows = []
    for m in masses:
        core = compute_physics_core(PhysicsCoreInputs(
            payload_mass_kg=m, altitude_m=tc.altitude_m,
            ground_velocity_n=v_n, ground_velocity_e=v_e, ground_velocity_u=0.0,
            servo_delay_s=tc.servo_delay_s,
        ))
        rows.append((m, core.t_flight_s, core.drop_distance_m))

    plt = _mpl()
    fig, axes = plt.subplots(1, 2, figsize=(10, 4))
    axes[0].plot([r[0] for r in rows], [r[1] for r in rows], "o-")
    axes[0].set_xlabel("payload_mass_kg (test points, not real measured masses)")
    axes[0].set_ylabel("flight_time_s")
    axes[0].set_title("Ballistic-only: mass has NO effect (a_gravity = g)")
    axes[1].plot([r[0] for r in rows], [r[2] for r in rows], "o-")
    axes[1].set_xlabel("payload_mass_kg (test points)")
    axes[1].set_ylabel("drop_distance_m")
    axes[1].set_title("Ballistic-only: drop distance unaffected by mass")
    fig.suptitle("Mass sensitivity — ballistic-only model (drag disabled: Cd/A UNKNOWN)")
    path = os.path.join(out_dir, "17_mass_sensitivity.png")
    fig.savefig(path, dpi=120, bbox_inches="tight")
    plt.close(fig)
    return path


def generate_acceptance_report(trials: List[MonteCarloTrial]) -> dict:
    metrics = compute_performance_metrics(trials)
    verdict = evaluate_acceptance(metrics)

    print("\n" + "=" * 60)
    print("PERFORMANCE REQUIREMENT: Impact Error (2D) <= "
          f"{config.MAX_ACCEPTABLE_IMPACT_ERROR_M} m (performance requirement, not a physics guarantee)")
    print("=" * 60)
    print(f"N trials              : {metrics['n_trials']}")
    print(f"Mean signed error [m]  : {metrics['mean_signed_error_m']:.3f}")
    print(f"MAE [m]                : {metrics['mae_m']:.3f}")
    print(f"RMSE [m]               : {metrics['rmse_m']:.3f}")
    print(f"Median AE [m]          : {metrics['median_ae_m']:.3f}")
    print(f"STD [m]                : {metrics['std_m']:.3f}")
    print(f"P95 abs error [m]      : {metrics['p95_abs_error_m']:.3f}")
    print(f"Max abs error [m]      : {metrics['max_abs_error_m']:.3f}")
    print(f"Within 5 m [%]         : {metrics['within_5m_pct']:.2f}")
    print(f"Outside 5 m [%]        : {metrics['outside_5m_pct']:.2f}")
    print(f"\nAcceptance policy (fixed BEFORE running this): {config.ACCEPTANCE_POLICY}")
    print(f"VERDICT: {verdict}")

    if verdict == "PASS":
        print("MAE, P95, and within-5m% all satisfy the acceptance policy.")
    elif verdict == "FAIL":
        print("At least one of MAE/P95/within-5m% failed the acceptance policy — "
              "reporting the real numbers above, not claiming <=5 m system-wide.")
    else:
        print("Not enough trials to draw a statistically meaningful PASS/FAIL conclusion.")

    return {"metrics": metrics, "verdict": verdict}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--trials", type=int, default=None, help="override config.TEST_CASE.monte_carlo_trials")
    parser.add_argument("--seed", type=int, default=None)
    parser.add_argument("--out-dir", default="plots")
    args = parser.parse_args()

    os.makedirs(args.out_dir, exist_ok=True)

    trials = run_monte_carlo(n_trials=args.trials, seed=args.seed)
    report = generate_acceptance_report(trials)

    produced = []
    produced.append(plot_release_distance_vs_trial(trials, args.out_dir))
    produced.append(plot_hist([t.required_release_distance_m for t in trials],
                               "Release Distance Distribution", "Required Release Distance [m]",
                               "02_release_distance_distribution.png", args.out_dir))
    produced.append(plot_hist([t.flight_time_s for t in trials],
                               "Flight-Time Distribution", "Flight Time [s]",
                               "03_flight_time_distribution.png", args.out_dir))
    produced.append(plot_hist([t.drop_distance_m for t in trials],
                               "Drop-Distance Distribution", "Drop Distance [m]",
                               "04_drop_distance_distribution.png", args.out_dir))

    plt = _mpl()
    fig, ax = plt.subplots()
    ax.plot([t.trial for t in trials], [t.signed_impact_error_m for t in trials], ".", ms=2)
    ax.axhline(0, color="k", linewidth=0.5)
    ax.set_xlabel("Trial")
    ax.set_ylabel("Signed Impact Error [m]  (+ = overshoot, - = undershoot, 0 = target)")
    ax.set_title("Signed Impact Error vs Trial")
    p = os.path.join(args.out_dir, "05_signed_impact_error_vs_trial.png")
    fig.savefig(p, dpi=120, bbox_inches="tight")
    plt.close(fig)
    produced.append(p)

    produced.append(plot_hist([t.absolute_impact_error_m for t in trials],
                               "Absolute Impact Error Distribution", "Absolute Impact Error (2D) [m]",
                               "06_absolute_impact_error_distribution.png", args.out_dir))

    print("\nNOTE: plots 07 (predicted vs actual impact) and 08/09 (before/after "
          "calibration) require real logged flight-test data with measured actual "
          "impact points — not produced here since no such CSV was supplied. Run "
          "train_model.py against a real flight-test CSV and extend this script "
          "with that data to generate them.")

    produced.append(plot_scatter_error_vs(trials, "ground_speed_mps", "Ground Speed [m/s]",
                                           "10_error_vs_ground_speed.png", args.out_dir))
    produced.append(plot_scatter_error_vs(trials, "air_speed_mps", "Air Speed [m/s]",
                                           "11_error_vs_air_speed.png", args.out_dir))
    produced.append(plot_scatter_error_vs(trials, "altitude_m", "Altitude [m]",
                                           "12_error_vs_altitude.png", args.out_dir))
    produced.append(plot_scatter_error_vs(trials, "payload_mass_kg", "Payload Mass [kg]",
                                           "13_error_vs_payload_mass.png", args.out_dir))
    produced.append(plot_scatter_error_vs(trials, "wind_speed_mps", "Wind Speed [m/s]",
                                           "14_error_vs_wind_speed.png", args.out_dir))
    produced.append(plot_scatter_error_vs(trials, "heading_deg", "Heading [deg]",
                                           "15_error_vs_heading.png", args.out_dir))
    produced.append(plot_scatter_error_vs(trials, "flight_path_angle_deg", "Flight-Path Angle [deg]",
                                           "16_error_vs_flight_path_angle.png", args.out_dir))

    produced.append(plot_sensitivity_mass(args.out_dir))

    for pid in (1, 2):
        box = TargetBox(**config.TARGETS[pid]["box"])
        produced.append(plot_target_box_with_impacts(trials, box, pid, args.out_dir))

    if config.TARGET_1["lat"] is None or config.TARGET_2["lat"] is None:
        print("\nNOTE: plot 20 (mission map) skipped — target lat/lon are UNKNOWN "
              "placeholders in config.py. Fill in real mission coordinates to enable it.")
    else:
        print("\nMission map generation with real target coordinates not yet wired "
              "to a real logged trajectory — supply one to extend this script.")

    print(f"\nWrote {len(produced)} plots to {args.out_dir}/")
    return 0 if report["verdict"] != "FAIL" else 1


if __name__ == "__main__":
    raise SystemExit(main())
