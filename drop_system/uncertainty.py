"""Uncertainty metadata and estimation utilities.

Every sigma used anywhere in this system carries provenance metadata
(UncertaintyEstimate). Test-case sigmas from config.TEST_UNCERTAINTY are
tagged source=ASSUMED/CONFIGURED (per spec section 3: "TEST ASSUMPTIONS,
not sensor specifications"). Sigmas estimated from real residual data
are tagged source=ESTIMATED/MEASURED, with sample_count and method
recorded so a caller can judge confidence.
"""

from __future__ import annotations

import math
import statistics
from dataclasses import dataclass, field
from typing import List, Optional, Sequence

from coordinate_utils import wrap_angle_deg

VALID_SOURCES = ("MEASURED", "ESTIMATED", "CONFIGURED", "ASSUMED", "UNKNOWN")


@dataclass
class UncertaintyEstimate:
    mean: Optional[float]
    sigma: Optional[float]
    sample_count: int
    method: str
    source: str  # one of VALID_SOURCES
    confidence: str  # "LOW" / "MEDIUM" / "HIGH" / "UNKNOWN"
    unit: str = ""

    def __post_init__(self):
        if self.source not in VALID_SOURCES:
            raise ValueError(f"invalid uncertainty source {self.source!r}")


def _confidence_from_sample_count(n: int) -> str:
    if n <= 0:
        return "UNKNOWN"
    if n < 10:
        return "LOW"
    if n < 50:
        return "MEDIUM"
    return "HIGH"


def estimate_parameter_uncertainty(samples: Sequence[float], unit: str = "",
                                    method: str = "sample_std") -> UncertaintyEstimate:
    """Estimate mean/sigma from a linear (non-circular) sample sequence.

    Returns source=ESTIMATED with sample_count and a confidence bucket
    derived purely from sample size (not a claim about sensor accuracy).
    Falls back to UNKNOWN with sigma=None if fewer than 2 samples.
    """
    n = len(samples)
    if n < 2:
        return UncertaintyEstimate(
            mean=(samples[0] if n == 1 else None),
            sigma=None, sample_count=n, method=method,
            source="UNKNOWN", confidence="UNKNOWN", unit=unit,
        )
    mean = statistics.fmean(samples)
    sigma = statistics.stdev(samples)
    return UncertaintyEstimate(
        mean=mean, sigma=sigma, sample_count=n, method=method,
        source="ESTIMATED", confidence=_confidence_from_sample_count(n), unit=unit,
    )


def estimate_residual_uncertainty(raw: Sequence[float], filtered: Sequence[float],
                                   unit: str = "") -> UncertaintyEstimate:
    """Sigma of (raw - filtered) residuals — separates sensor-noise-like
    variation from the underlying smoothed signal (spec sections 66-68).
    Requires len(raw) == len(filtered).
    """
    if len(raw) != len(filtered):
        raise ValueError("raw and filtered sequences must be same length")
    residuals = [r - f for r, f in zip(raw, filtered)]
    return estimate_parameter_uncertainty(residuals, unit=unit, method="raw_minus_filtered_residual")


def estimate_circular_uncertainty(samples_deg: Sequence[float], reference_deg: Sequence[float],
                                   unit: str = "deg") -> UncertaintyEstimate:
    """Circular-statistics sigma for angular residuals (heading, wind
    direction, flight-path angle if angular). Uses wrap_angle_deg so a
    358 vs 2 degree pair is treated as a 4 degree difference, not 356
    (spec section 69).
    """
    if len(samples_deg) != len(reference_deg):
        raise ValueError("sample and reference sequences must be same length")
    n = len(samples_deg)
    if n < 2:
        return UncertaintyEstimate(
            mean=None, sigma=None, sample_count=n, method="circular_residual",
            source="UNKNOWN", confidence="UNKNOWN", unit=unit,
        )
    residuals = [wrap_angle_deg(s - r) for s, r in zip(samples_deg, reference_deg)]
    mean = statistics.fmean(residuals)
    sigma = statistics.stdev(residuals)
    return UncertaintyEstimate(
        mean=mean, sigma=sigma, sample_count=n, method="circular_residual",
        source="ESTIMATED", confidence=_confidence_from_sample_count(n), unit=unit,
    )


def configured_uncertainty(value: float, unit: str = "", source: str = "CONFIGURED") -> UncertaintyEstimate:
    """Wrap a fixed CONFIGURED/ASSUMED sigma (e.g. from config.TEST_UNCERTAINTY)
    in the same metadata structure, so downstream code doesn't need to
    special-case "is this a real estimate or a test constant".
    """
    return UncertaintyEstimate(
        mean=None, sigma=value, sample_count=0, method="fixed_configuration",
        source=source, confidence="UNKNOWN", unit=unit,
    )


@dataclass
class DistributionSummary:
    mean: float
    median: float
    std: float
    p05: float
    p50: float
    p95: float
    n: int


def summarize_distribution(values: Sequence[float]) -> DistributionSummary:
    """mean/median/std/P05/P50/P95 summary used for Monte Carlo output
    (spec section 77/83).
    """
    if not values:
        raise ValueError("cannot summarize an empty sequence")
    ordered = sorted(values)
    n = len(ordered)

    def percentile(p: float) -> float:
        if n == 1:
            return ordered[0]
        idx = p / 100.0 * (n - 1)
        lo = int(math.floor(idx))
        hi = int(math.ceil(idx))
        if lo == hi:
            return ordered[lo]
        frac = idx - lo
        return ordered[lo] * (1 - frac) + ordered[hi] * frac

    mean = statistics.fmean(values)
    median = statistics.median(values)
    std = statistics.stdev(values) if n > 1 else 0.0
    return DistributionSummary(
        mean=mean, median=median, std=std,
        p05=percentile(5), p50=percentile(50), p95=percentile(95), n=n,
    )
