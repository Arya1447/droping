#!/usr/bin/env python3
"""CLI: train/compare empirical correction models on a flight-test CSV
and write the selected model's parameters to model_parameters.json
(spec sections 84-89, 124-126, 142).

Usage:
    python3 train_model.py --csv example_drop_tests.csv --out model_parameters.json

NOTE: example_drop_tests.csv shipped with this project is SYNTHETIC
example data for exercising this pipeline end-to-end — it is NOT real
flight-test data and any metrics produced from it must not be reported
as real system performance. Replace it with real logged flights before
drawing any accuracy conclusion.
"""

from __future__ import annotations

import argparse
import json
import pickle
import sys
from datetime import datetime, timezone

import calibration
import config


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--csv", default="example_drop_tests.csv")
    parser.add_argument("--out", default="model_parameters.json")
    parser.add_argument("--model-pickle-out", default="model_parameters.pkl")
    parser.add_argument("--k-folds", type=int, default=5)
    args = parser.parse_args()

    df = calibration.load_flight_test_data(args.csv)
    n_valid = (df["row_status"] == "VALID").sum()
    n_suspicious = (df["row_status"] == "SUSPICIOUS").sum()
    n_invalid = (df["row_status"] == "INVALID").sum()
    print(f"Loaded {len(df)} rows: {n_valid} VALID, {n_suspicious} SUSPICIOUS, {n_invalid} INVALID")

    if n_valid < args.k_folds:
        print(f"INSUFFICIENT_DATA: only {n_valid} VALID rows, need >= {args.k_folds} for k-fold CV.")
        return 1

    results = calibration.compare_models(df, k_folds=args.k_folds,
                                          max_acceptable_m=config.MAX_ACCEPTABLE_IMPACT_ERROR_M)

    print("\nModel comparison (validation metrics, never training error):")
    header = f"{'model':28s} {'MAE':>7s} {'RMSE':>7s} {'MedAE':>7s} {'STD':>7s} {'P95':>7s} {'Max':>7s} {'W/5m%':>7s}"
    print(header)
    for name, m in results.items():
        print(f"{name:28s} {m.mae_m:7.2f} {m.rmse_m:7.2f} {m.median_ae_m:7.2f} "
              f"{m.std_m:7.2f} {m.p95_abs_error_m:7.2f} {m.max_abs_error_m:7.2f} {m.within_5m_pct:7.1f}")

    best_name = calibration.select_best_model(results)
    print(f"\nSelected (lowest validation MAE): {best_name}")

    best_metrics = results[best_name]
    mae_ok = best_metrics.mae_m <= config.MAX_ACCEPTABLE_IMPACT_ERROR_M
    p95_ok = best_metrics.p95_abs_error_m <= config.MAX_ACCEPTABLE_IMPACT_ERROR_M
    if mae_ok and p95_ok:
        print(f"MAE ({best_metrics.mae_m:.2f} m) and P95 ({best_metrics.p95_abs_error_m:.2f} m) "
              f"both <= {config.MAX_ACCEPTABLE_IMPACT_ERROR_M} m.")
    else:
        print(f"WARNING: does not fully meet the <= {config.MAX_ACCEPTABLE_IMPACT_ERROR_M} m requirement — "
              f"MAE {'OK' if mae_ok else 'FAIL'}, P95 {'OK' if p95_ok else 'FAIL'}. "
              "Reporting actual numbers, not claiming pass.")

    model = calibration.fit_final_model(df, best_name)
    if model is not None:
        with open(args.model_pickle_out, "wb") as f:
            pickle.dump(model, f)

    metadata = {
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "source_csv": args.csv,
        "csv_is_synthetic_example": args.csv == "example_drop_tests.csv",
        "n_rows_total": len(df),
        "n_rows_valid": int(n_valid),
        "n_rows_suspicious": int(n_suspicious),
        "n_rows_invalid": int(n_invalid),
        "selected_model": best_name,
        "model_pickle_path": args.model_pickle_out if model is not None else None,
        "feature_order": calibration.FEATURE_COLUMNS,
        "performance_requirement_m": config.MAX_ACCEPTABLE_IMPACT_ERROR_M,
        "acceptance_policy": config.ACCEPTANCE_POLICY,
        "model_comparison": {
            name: {
                "mae_m": m.mae_m, "rmse_m": m.rmse_m, "median_ae_m": m.median_ae_m,
                "std_m": m.std_m, "p95_abs_error_m": m.p95_abs_error_m,
                "max_abs_error_m": m.max_abs_error_m, "within_5m_pct": m.within_5m_pct,
                "inference_time_s": m.inference_time_s, "n_samples": m.n_samples,
            }
            for name, m in results.items()
        },
        "payload_mass_source": config.PAYLOAD_MASS_SOURCE,
    }
    with open(args.out, "w") as f:
        json.dump(metadata, f, indent=2)
    print(f"\nWrote {args.out}" + (f" and {args.model_pickle_out}" if model is not None else ""))
    return 0


if __name__ == "__main__":
    sys.exit(main())
