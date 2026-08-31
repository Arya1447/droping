"""Telemetry freshness tracking and aggregate health state.

Per-channel model: each telemetry quantity (GPS, altitude, ground speed,
airspeed, heartbeat, wind, heading, mission, servo feedback) has a
last-update timestamp. Freshness is evaluated as
`age = current_time - last_update_time` against a configurable timeout.

Criticality (spec sections 52/159, final):
    CRITICAL:     GPS, altitude, ground speed, airspeed, heartbeat.
                  Stale/invalid -> hard warning, release blocked.
    NON-CRITICAL: wind, heading, mission auxiliary data, servo feedback.
                  Stale -> WARNING only, unless the currently active
                  model has made that data a mandatory dependency.

Recovery: after a critical channel returns from stale, `release_allowed`
does not immediately re-arm — the channel must stay fresh for
config.TELEMETRY_RECOVERY_CYCLES consecutive health checks first (spec
section 59), enforced by TelemetryHealth.update().
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional

import config

CRITICAL_CHANNELS = ("gps", "altitude", "ground_speed", "airspeed", "heartbeat")
NON_CRITICAL_CHANNELS = ("wind", "heading", "mission", "servo_feedback")

DEFAULT_TIMEOUTS: Dict[str, float] = {
    "gps": config.GPS_TIMEOUT_S,
    "altitude": config.ALTITUDE_TIMEOUT_S,
    "ground_speed": config.GROUND_SPEED_TIMEOUT_S,
    "airspeed": config.AIRSPEED_TIMEOUT_S,
    "heartbeat": config.HEARTBEAT_TIMEOUT_S,
    "wind": config.WIND_TIMEOUT_S,
    "heading": config.HEADING_TIMEOUT_S,
    "mission": config.MISSION_TIMEOUT_S,
    "servo_feedback": config.SERVO_FEEDBACK_TIMEOUT_S,
}


@dataclass
class TelemetryChannel:
    name: str
    timeout_s: float
    last_update_time: Optional[float] = None
    value: Optional[object] = None

    def touch(self, value: object, now: Optional[float] = None) -> None:
        self.value = value
        self.last_update_time = now if now is not None else time.monotonic()

    def get_data_age(self, now: Optional[float] = None) -> Optional[float]:
        if self.last_update_time is None:
            return None
        now = now if now is not None else time.monotonic()
        return now - self.last_update_time

    def is_data_fresh(self, now: Optional[float] = None) -> bool:
        age = self.get_data_age(now)
        if age is None:
            return False
        return age <= self.timeout_s


@dataclass
class TelemetryHealth:
    """Aggregates all telemetry channels into HEALTHY/DEGRADED/CRITICAL."""

    channels: Dict[str, TelemetryChannel] = field(default_factory=dict)
    _fresh_streak: Dict[str, int] = field(default_factory=dict)
    _recovered: Dict[str, bool] = field(default_factory=dict)

    def __post_init__(self):
        if not self.channels:
            for name in CRITICAL_CHANNELS + NON_CRITICAL_CHANNELS:
                self.channels[name] = TelemetryChannel(name=name, timeout_s=DEFAULT_TIMEOUTS[name])
        for name in self.channels:
            self._fresh_streak.setdefault(name, 0)
            # Optimistic initial state: a channel that has never gone
            # stale needs no recovery streak. The recovery-cycle gate
            # (spec section 59) only engages after an actual stale
            # episode is observed below.
            self._recovered.setdefault(name, True)

    def touch(self, channel: str, value: object, now: Optional[float] = None) -> None:
        if channel not in self.channels:
            self.channels[channel] = TelemetryChannel(name=channel, timeout_s=DEFAULT_TIMEOUTS.get(channel, 1.0))
        self.channels[channel].touch(value, now)

    def evaluate(self, now: Optional[float] = None) -> "HealthReport":
        now = now if now is not None else time.monotonic()
        critical_stale: List[str] = []
        non_critical_stale: List[str] = []

        for name, ch in self.channels.items():
            fresh = ch.is_data_fresh(now)
            if fresh:
                self._fresh_streak[name] = self._fresh_streak.get(name, 0) + 1
            else:
                self._fresh_streak[name] = 0
                self._recovered[name] = False  # entering/continuing a stale episode

            if name in CRITICAL_CHANNELS:
                if not fresh:
                    critical_stale.append(name)
                elif self._recovered[name]:
                    pass  # fresh and never needed recovery gating -> healthy
                elif self._fresh_streak[name] >= config.TELEMETRY_RECOVERY_CYCLES:
                    # Fresh for N consecutive cycles after a stale
                    # episode (spec section 59) -> trusted again.
                    self._recovered[name] = True
                else:
                    critical_stale.append(name)  # fresh now, still recovering
            elif name in NON_CRITICAL_CHANNELS:
                if not fresh:
                    non_critical_stale.append(name)

        if critical_stale:
            state = "CRITICAL"
        elif non_critical_stale:
            state = "DEGRADED"
        else:
            state = "HEALTHY"

        ages = {name: ch.get_data_age(now) for name, ch in self.channels.items()}
        return HealthReport(
            state=state,
            critical_stale=critical_stale,
            non_critical_stale=non_critical_stale,
            ages_s=ages,
        )


@dataclass
class HealthReport:
    state: str  # HEALTHY / DEGRADED / CRITICAL
    critical_stale: List[str]
    non_critical_stale: List[str]
    ages_s: Dict[str, Optional[float]]

    def release_blocked_reasons(self) -> List[str]:
        return [f"{name.upper()}_STALE" for name in self.critical_stale]

    def format_debug(self) -> str:
        lines = []
        for name, age in self.ages_s.items():
            if age is None:
                status = "NO_DATA"
                age_str = "n/a"
            else:
                is_critical = name in CRITICAL_CHANNELS
                timeout = DEFAULT_TIMEOUTS.get(name, 1.0)
                stale = age > timeout
                status = "STALE" if stale else "OK"
                if stale and is_critical:
                    status = "!!! CRITICAL !!!"
                age_str = f"{age:.2f}s"
            lines.append(f"{name:15s}: {status:20s} age={age_str}")
        lines.append(f"OVERALL: {self.state}")
        return "\n".join(lines)
