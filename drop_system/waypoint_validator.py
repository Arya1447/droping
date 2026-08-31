"""Geometric waypoint-crossing validation (spec sections 35-37, 155).

`MISSION_CURRENT.seq >= required_waypoint` is deliberately NOT used as
the (sole) crossing condition — it is a mission-sequence signal, not a
spatial one, and does not prove the aircraft has actually flown past
the waypoint's location. Instead this module projects the aircraft's
position onto the segment between the previous and current waypoint and
checks for a sign change in the along-segment projection.

Once a waypoint is detected as passed, the result latches True and never
reverts to False for the lifetime of the WaypointValidator instance,
protecting against transient telemetry noise (spec section 37).
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Optional, Tuple


def _dot(a: Tuple[float, float], b: Tuple[float, float]) -> float:
    return a[0] * b[0] + a[1] * b[1]


def _sub(a: Tuple[float, float], b: Tuple[float, float]) -> Tuple[float, float]:
    return a[0] - b[0], a[1] - b[1]


def _norm(a: Tuple[float, float]) -> float:
    return math.hypot(a[0], a[1])


def waypoint_spatially_passed(
    p_prev: Tuple[float, float], p_curr: Tuple[float, float],
    w_prev: Tuple[float, float], w_current: Tuple[float, float],
) -> bool:
    """Segment-projection crossing test (spec section 35/155):

        d = W_current - W_prev
        u = d / |d|
        r_prev = P_prev - W_prev ;  s_prev = r_prev . u
        r_curr = P_curr - W_prev ;  s_curr = r_curr . u
        passed iff s_prev < s_waypoint AND s_curr >= s_waypoint

    where s_waypoint = |d| (the waypoint's own projection onto the
    segment, i.e. the segment endpoint).
    """
    d = _sub(w_current, w_prev)
    seg_len = _norm(d)
    if seg_len < 1e-6:
        # Degenerate segment (waypoints coincide) — cannot determine a
        # meaningful crossing direction; fall back to distance-to-point.
        dist = _norm(_sub(p_curr, w_current))
        return dist < 1.0  # ASSUMED 1 m capture radius for degenerate case only

    u = (d[0] / seg_len, d[1] / seg_len)
    s_waypoint = seg_len
    s_prev = _dot(_sub(p_prev, w_prev), u)
    s_curr = _dot(_sub(p_curr, w_prev), u)
    return s_prev < s_waypoint <= s_curr


def turn_angle_deg(u_in: Tuple[float, float], u_out: Tuple[float, float]) -> float:
    """theta_turn = acos(clamp(u_in . u_out, -1, 1)), for validating that
    the aircraft has actually exited a turning waypoint (spec section 36).
    """
    dot = max(-1.0, min(1.0, _dot(u_in, u_out)))
    return math.degrees(math.acos(dot))


@dataclass
class WaypointValidator:
    required_waypoint_seq: int
    _passed: bool = False
    _prev_position: Optional[Tuple[float, float]] = None
    _last_mission_seq: Optional[int] = None

    def update(
        self,
        mission_seq: int,
        current_position: Tuple[float, float],
        waypoint_prev_position: Optional[Tuple[float, float]],
        waypoint_current_position: Optional[Tuple[float, float]],
    ) -> bool:
        """Call once per telemetry cycle. Returns the latched
        `waypoint_passed` state.

        mission_seq: MISSION_CURRENT.seq, used only as a corroborating
            signal (must have reached at least required_waypoint_seq),
            never as the sole crossing condition.
        current_position: aircraft (N, E) this cycle.
        waypoint_prev_position / waypoint_current_position: (N, E) of
            the waypoint immediately before, and the required waypoint
            itself, from MISSION_ITEM_INT. If either is unavailable,
            spatial crossing cannot be evaluated this cycle (no latch
            change, no crash).
        """
        self._last_mission_seq = mission_seq

        if self._passed:
            return True  # latched

        if mission_seq < self.required_waypoint_seq:
            self._prev_position = current_position
            return False

        if (
            self._prev_position is not None
            and waypoint_prev_position is not None
            and waypoint_current_position is not None
        ):
            if waypoint_spatially_passed(
                self._prev_position, current_position,
                waypoint_prev_position, waypoint_current_position,
            ):
                self._passed = True

        self._prev_position = current_position
        return self._passed

    @property
    def passed(self) -> bool:
        return self._passed

    def reset(self) -> None:
        """Explicit reset for a new mission instance only — never called
        automatically from telemetry noise.
        """
        self._passed = False
        self._prev_position = None
        self._last_mission_seq = None
