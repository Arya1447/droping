"""Lightweight filters (EMA, median) for telemetry smoothing.

Trade-off (spec section 62): a larger window / smaller EMA alpha reduces
noise but adds latency — the filtered value lags the true signal by
roughly one time-constant. Because release timing depends on real-time
ground speed / altitude / heading, filter parameters here are kept small
(config.EMA_ALPHA_* around 0.2-0.3, median window 3) to bound that lag;
this is a CONFIGURED trade-off, not a measured optimum.
"""

from __future__ import annotations

from collections import deque
from typing import Deque, Optional

from coordinate_utils import wrap_angle_deg


class EMAFilter:
    """Exponential moving average: y_k = alpha*x_k + (1-alpha)*y_{k-1}."""

    def __init__(self, alpha: float, initial: Optional[float] = None):
        if not (0.0 < alpha <= 1.0):
            raise ValueError(f"alpha must be in (0, 1], got {alpha}")
        self.alpha = alpha
        self._value: Optional[float] = initial

    def update(self, x: float) -> float:
        if self._value is None:
            self._value = x
        else:
            self._value = self.alpha * x + (1.0 - self.alpha) * self._value
        return self._value

    @property
    def value(self) -> Optional[float]:
        return self._value

    def reset(self) -> None:
        self._value = None


class CircularEMAFilter:
    """EMA for angular quantities (degrees), using wrapped residuals to
    avoid the 359->1 degree discontinuity.
    """

    def __init__(self, alpha: float, initial: Optional[float] = None):
        if not (0.0 < alpha <= 1.0):
            raise ValueError(f"alpha must be in (0, 1], got {alpha}")
        self.alpha = alpha
        self._value: Optional[float] = initial

    def update(self, x_deg: float) -> float:
        if self._value is None:
            self._value = x_deg % 360.0
        else:
            residual = wrap_angle_deg(x_deg - self._value)
            self._value = (self._value + self.alpha * residual) % 360.0
        return self._value

    @property
    def value(self) -> Optional[float]:
        return self._value

    def reset(self) -> None:
        self._value = None


class MedianFilter:
    """Sliding-window median filter."""

    def __init__(self, window: int):
        if window < 1:
            raise ValueError(f"window must be >= 1, got {window}")
        self.window = window
        self._buffer: Deque[float] = deque(maxlen=window)

    def update(self, x: float) -> float:
        self._buffer.append(x)
        ordered = sorted(self._buffer)
        mid = len(ordered) // 2
        if len(ordered) % 2 == 1:
            return ordered[mid]
        return 0.5 * (ordered[mid - 1] + ordered[mid])

    @property
    def value(self) -> Optional[float]:
        if not self._buffer:
            return None
        ordered = sorted(self._buffer)
        mid = len(ordered) // 2
        if len(ordered) % 2 == 1:
            return ordered[mid]
        return 0.5 * (ordered[mid - 1] + ordered[mid])

    def reset(self) -> None:
        self._buffer.clear()
