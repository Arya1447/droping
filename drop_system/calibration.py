"""Empirical correction models fitted from real flight-test data
(spec sections 84-89, 124-125, 134, 141-142).

Offline-only (spec section 117): imports scikit-learn/pandas lazily so
the live runtime never pays this cost.

Model comparison, in increasing complexity (spec section 84 — complexity
is never chosen automatically, only by validation performance):

    physics_only              — zero correction (baseline = whatever
                                 error the physics-only prediction
                                 already produced, i.e. the CSV's own
                                 `signed_impact_error` column).
    physics_empirical_ridge   — Ridge regression correction term.
    physics_empirical_poly2   — degree-2 polynomial regression correction.
    pure_ml_random_forest     — RandomForestRegressor correction.

All correction models predict SIGNED error (spec section 85: absolute
error alone cannot teach a release-direction correction) as a function
of:

    correction = f(payload_mass, ground_speed, air_speed, altitude,
                    wind_speed, wind_direction, heading,
                    flight_path_angle, servo_delay)

Corrected residual = signed_impact_error - predicted_correction;
metrics are computed on out-of-fold cross-validated predictions, never
on training-set error (spec section 86).
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List, Optional

FEATURE_COLUMNS = [
    "payload_mass", "ground_speed", "air_speed", "altitude",
    "wind_speed", "wind_direction", "heading", "flight_path_angle",
]
TARGET_COLUMN = "signed_impact_error"

REQUIRED_CSV_COLUMNS = [
    "timestamp", "aircraft_lat", "aircraft_lon", "altitude",
    "ground_speed", "air_speed", "ground_track", "heading", "flight_path_angle",
    "wind_speed", "wind_direction", "payload_mass", "target_distance",
    "release_distance", "predicted_drop_distance", "predicted_release_distance",
    "actual_impact_lat", "actual_impact_lon", "actual_impact_along_track",
    "actual_impact_cross_track", "signed_impact_error", "impact_error",
    "payload_id", "release_state",
]


def load_flight_test_data(csv_path: str):
    """Load a flight-test CSV (spec section 97 schema) and flag each row
    VALID / SUSPICIOUS / INVALID (spec section 88). Requires pandas.
    """
    import pandas as pd

    df = pd.read_csv(csv_path)
    missing = [c for c in REQUIRED_CSV_COLUMNS if c not in df.columns]
    if missing:
        raise ValueError(f"flight-test CSV missing required columns: {missing}")

    df["row_status"] = "VALID"
    df["row_status_reason"] = ""

    # Physical-bounds check.
    bad_physical = (df["altitude"] <= 0) | (df["ground_speed"] <= 0) | (df["air_speed"] <= 0) | (df["payload_mass"] <= 0)
    df.loc[bad_physical, "row_status"] = "INVALID"
    df.loc[bad_physical, "row_status_reason"] = "non-physical altitude/speed/mass"

    # IQR-based outlier flag on impact_error, applied only to rows not
    # already INVALID.
    valid_mask = df["row_status"] == "VALID"
    if valid_mask.sum() >= 4:
        q1 = df.loc[valid_mask, "impact_error"].quantile(0.25)
        q3 = df.loc[valid_mask, "impact_error"].quantile(0.75)
        iqr = q3 - q1
        lower, upper = q1 - 1.5 * iqr, q3 + 1.5 * iqr
        suspicious = valid_mask & ((df["impact_error"] < lower) | (df["impact_error"] > upper))
        df.loc[suspicious, "row_status"] = "SUSPICIOUS"
        df.loc[suspicious, "row_status_reason"] = f"impact_error outside IQR bounds [{lower:.2f}, {upper:.2f}]"

    return df


@dataclass
class ModelMetrics:
    model_name: str
    mae_m: float
    rmse_m: float
    median_ae_m: float
    std_m: float
    p95_abs_error_m: float
    max_abs_error_m: float
    within_5m_pct: float
    inference_time_s: float
    n_samples: int


def _cv_metrics(y_true, y_pred_cv, model_name: str, inference_time_s: float,
                 max_acceptable_m: float) -> ModelMetrics:
    import math
    import statistics

    abs_err = [abs(t - p) for t, p in zip(y_true, y_pred_cv)]
    n = len(abs_err)
    mae = statistics.fmean(abs_err)
    rmse = math.sqrt(sum(e ** 2 for e in abs_err) / n)
    median_ae = statistics.median(abs_err)
    std = statistics.stdev(abs_err) if n > 1 else 0.0
    ordered = sorted(abs_err)
    p95_idx = min(n - 1, int(round(0.95 * (n - 1))))
    p95 = ordered[p95_idx]
    within_pct = 100.0 * sum(1 for e in abs_err if e <= max_acceptable_m) / n
    return ModelMetrics(
        model_name=model_name, mae_m=mae, rmse_m=rmse, median_ae_m=median_ae,
        std_m=std, p95_abs_error_m=p95, max_abs_error_m=max(abs_err),
        within_5m_pct=within_pct, inference_time_s=inference_time_s, n_samples=n,
    )


def compare_models(df, k_folds: int = 5, max_acceptable_m: float = 5.0,
                    min_samples_per_leaf: int = 3, max_depth: int = 4) -> Dict[str, ModelMetrics]:
    """Fit + cross-validate all candidate models on VALID rows only.
    Returns {model_name: ModelMetrics}. Requires scikit-learn.

    Regularization choices (Ridge default alpha=1.0, RandomForest
    max_depth/min_samples_leaf capped) follow spec section 87: for a
    small dataset, prefer simpler/regularized models over an
    unconstrained fit.
    """
    import time
    import numpy as np
    from sklearn.ensemble import RandomForestRegressor
    from sklearn.linear_model import Ridge
    from sklearn.model_selection import KFold, cross_val_predict
    from sklearn.pipeline import make_pipeline
    from sklearn.preprocessing import PolynomialFeatures, StandardScaler

    valid = df[df["row_status"] == "VALID"].copy()
    n = len(valid)
    if n < k_folds:
        raise ValueError(
            f"only {n} VALID rows available; need at least k_folds={k_folds} "
            "for cross-validation. Collect more flight-test data."
        )

    x = valid[FEATURE_COLUMNS].to_numpy(dtype=float)
    y = valid[TARGET_COLUMN].to_numpy(dtype=float)
    y_physics_only = valid["signed_impact_error"].to_numpy(dtype=float)

    kf = KFold(n_splits=k_folds, shuffle=True, random_state=42)

    results: Dict[str, ModelMetrics] = {}

    # A. Physics-only: correction = 0, error is whatever the physics
    # model already produced (already logged as signed_impact_error).
    results["physics_only"] = _cv_metrics(y_physics_only, [0.0] * n, "physics_only", 0.0, max_acceptable_m)

    def run_model(name: str, estimator) -> None:
        t0 = time.perf_counter()
        pred_correction = cross_val_predict(estimator, x, y, cv=kf)
        elapsed = (time.perf_counter() - t0) / max(n, 1)
        corrected_error = y_physics_only - pred_correction
        results[name] = _cv_metrics(np.zeros_like(corrected_error), corrected_error, name, elapsed, max_acceptable_m)

    run_model("physics_empirical_ridge", Ridge(alpha=1.0))
    run_model("physics_empirical_poly2", make_pipeline(
        StandardScaler(), PolynomialFeatures(degree=2, include_bias=False), Ridge(alpha=1.0)
    ))
    run_model("pure_ml_random_forest", RandomForestRegressor(
        n_estimators=100, max_depth=max_depth, min_samples_leaf=min_samples_per_leaf, random_state=42
    ))

    return results


def select_best_model(results: Dict[str, ModelMetrics]) -> str:
    """Selection is by validation MAE only — never by training error,
    never by defaulting to the most complex candidate (spec section
    84/125).
    """
    return min(results, key=lambda name: results[name].mae_m)


def fit_final_model(df, model_name: str, min_samples_per_leaf: int = 3, max_depth: int = 4):
    """Fit the selected model on ALL VALID rows (not just CV folds) for
    deployment. Returns a fitted estimator, or None for physics_only.
    """
    from sklearn.ensemble import RandomForestRegressor
    from sklearn.linear_model import Ridge
    from sklearn.pipeline import make_pipeline
    from sklearn.preprocessing import PolynomialFeatures, StandardScaler

    valid = df[df["row_status"] == "VALID"]
    x = valid[FEATURE_COLUMNS].to_numpy(dtype=float)
    y = valid[TARGET_COLUMN].to_numpy(dtype=float)

    if model_name == "physics_only":
        return None
    if model_name == "physics_empirical_ridge":
        model = Ridge(alpha=1.0)
    elif model_name == "physics_empirical_poly2":
        model = make_pipeline(StandardScaler(), PolynomialFeatures(degree=2, include_bias=False), Ridge(alpha=1.0))
    elif model_name == "pure_ml_random_forest":
        model = RandomForestRegressor(n_estimators=100, max_depth=max_depth,
                                       min_samples_leaf=min_samples_per_leaf, random_state=42)
    else:
        raise ValueError(f"unknown model_name {model_name!r}")

    model.fit(x, y)
    return model


def make_correction_function(model, feature_order: List[str] = FEATURE_COLUMNS):
    """Wrap a fitted sklearn estimator into the
    `correction(**kwargs) -> float` signature prediction.py expects."""
    if model is None:
        return None

    def correction(**kwargs) -> float:
        row = [[
            kwargs.get("payload_mass_kg", 0.0),
            kwargs.get("ground_speed_mps", 0.0),
            kwargs.get("air_speed_mps", 0.0) or 0.0,
            kwargs.get("altitude_m", 0.0),
            kwargs.get("wind_speed_mps", 0.0) or 0.0,
            kwargs.get("wind_direction_deg", 0.0) or 0.0,
            kwargs.get("heading_deg", 0.0) or 0.0,
            kwargs.get("flight_path_angle_deg", 0.0) or 0.0,
        ]]
        return float(model.predict(row)[0])

    return correction
