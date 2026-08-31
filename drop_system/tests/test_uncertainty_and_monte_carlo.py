import math

import config
from monte_carlo import compute_performance_metrics, evaluate_acceptance, run_monte_carlo
from prediction import PredictionStabilityTracker
from uncertainty import estimate_circular_uncertainty, estimate_parameter_uncertainty, summarize_distribution


def test_estimate_parameter_uncertainty_basic():
    samples = [10.0, 10.1, 9.9, 10.05, 9.95]
    est = estimate_parameter_uncertainty(samples, unit="m/s")
    assert est.source == "ESTIMATED"
    assert est.mean == math.fsum(samples) / len(samples)
    assert est.sigma is not None and est.sigma > 0
    assert est.sample_count == 5


def test_estimate_parameter_uncertainty_insufficient_samples():
    est = estimate_parameter_uncertainty([5.0])
    assert est.source == "UNKNOWN"
    assert est.sigma is None


def test_circular_uncertainty_handles_wraparound():
    samples = [358.0, 2.0, 359.0, 1.0]
    reference = [0.0, 0.0, 0.0, 0.0]
    est = estimate_circular_uncertainty(samples, reference)
    # residuals should be small (near 0), not ~358 due to wrap
    assert abs(est.mean) < 10.0


def test_summarize_distribution_percentiles():
    values = list(range(1, 101))  # 1..100
    summary = summarize_distribution(values)
    assert summary.p50 == 50.5
    assert summary.n == 100


def test_monte_carlo_runs_and_returns_trials():
    trials = run_monte_carlo(n_trials=200, seed=1)
    assert len(trials) == 200
    for t in trials:
        assert t.flight_time_s > 0
        assert t.absolute_impact_error_m >= 0


def test_monte_carlo_performance_metrics_shape():
    trials = run_monte_carlo(n_trials=500, seed=2)
    metrics = compute_performance_metrics(trials)
    for key in ("mae_m", "rmse_m", "median_ae_m", "std_m", "p95_abs_error_m",
                "max_abs_error_m", "within_5m_pct", "outside_5m_pct"):
        assert key in metrics
    assert 0.0 <= metrics["within_5m_pct"] <= 100.0
    assert metrics["outside_5m_pct"] == 100.0 - metrics["within_5m_pct"]


def test_acceptance_insufficient_data():
    trials = run_monte_carlo(n_trials=5, seed=3)
    metrics = compute_performance_metrics(trials)
    verdict = evaluate_acceptance(metrics)
    assert verdict == "INSUFFICIENT_DATA"


def test_prediction_stability_tracker_requires_consecutive_stable_cycles():
    tracker = PredictionStabilityTracker(stable_cycles=3, max_change_m=1.0)
    assert tracker.update(100.0) is False  # only 1 sample
    assert tracker.update(100.5) is False  # only 2 samples
    assert tracker.update(100.9) is True   # 3 samples, deltas <= 1.0


def test_prediction_stability_tracker_detects_instability():
    tracker = PredictionStabilityTracker(stable_cycles=3, max_change_m=1.0)
    tracker.update(100.0)
    tracker.update(105.0)  # jump > 1.0
    assert tracker.update(105.2) is False
