"""Per-payload state machine (spec sections 38-41, 156, 161).

Two independent instances (one per payload) must exist — releasing
payload 1 must never affect payload 2's state, and vice versa. Gate
evaluation collects every failing condition into a reason list so a
HOLD is always explainable, never a silent no-op.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import List, Optional


class PayloadState(Enum):
    ARMED = "ARMED"
    APPROACH = "APPROACH"
    WAIT_WAYPOINT = "WAIT_WAYPOINT"
    WAYPOINT_PASSED = "WAYPOINT_PASSED"
    TARGET_VALIDATION = "TARGET_VALIDATION"
    RELEASE_WINDOW = "RELEASE_WINDOW"
    RELEASE = "RELEASE"
    DONE = "DONE"


# Linear progression; a payload can only move forward, never backward,
# except that failing an intermediate gate simply holds it in place
# (does not regress it to an earlier state).
_ORDER = [
    PayloadState.ARMED,
    PayloadState.APPROACH,
    PayloadState.WAIT_WAYPOINT,
    PayloadState.WAYPOINT_PASSED,
    PayloadState.TARGET_VALIDATION,
    PayloadState.RELEASE_WINDOW,
    PayloadState.RELEASE,
    PayloadState.DONE,
]


@dataclass
class GateInputs:
    """All boolean/quality inputs a release decision depends on. Every
    field must be explicitly supplied by the caller (prediction.py) —
    no field defaults to True, so a missing/unwired input fails closed.
    """
    gps_valid: bool = False
    altitude_valid: bool = False
    ground_speed_valid: bool = False
    airspeed_valid: bool = False
    heartbeat_valid: bool = False
    ekf_valid: bool = False
    geofence_valid: bool = False

    target_valid: bool = False
    target_box_valid: bool = False

    waypoint_passed: bool = False

    corridor_valid: bool = False
    target_ahead: bool = False

    prediction_valid: bool = False
    predicted_impact_inside_box: bool = False

    release_distance_valid: bool = False
    release_window_valid: bool = False
    prediction_stable: bool = False

    servo_mapping_valid: bool = False
    servo_safety_valid: bool = False

    live_release_enabled: bool = False


_GATE_REASON_MAP = [
    ("gps_valid", "GPS_INVALID"),
    ("altitude_valid", "ALTITUDE_INVALID"),
    ("ground_speed_valid", "GROUND_SPEED_INVALID"),
    ("airspeed_valid", "AIRSPEED_INVALID"),
    ("heartbeat_valid", "HEARTBEAT_INVALID"),
    ("ekf_valid", "EKF_INVALID"),
    ("geofence_valid", "OUTSIDE_GEOFENCE"),
    ("target_valid", "TARGET_INVALID"),
    ("target_box_valid", "TARGET_BOX_INVALID"),
    ("waypoint_passed", "WAYPOINT_NOT_PASSED"),
    ("corridor_valid", "CORRIDOR_INVALID"),
    ("target_ahead", "TARGET_NOT_AHEAD"),
    ("prediction_valid", "PREDICTION_INVALID"),
    ("predicted_impact_inside_box", "PREDICTED_IMPACT_OUTSIDE_BOX"),
    ("release_distance_valid", "RELEASE_DISTANCE_INVALID"),
    ("release_window_valid", "RELEASE_WINDOW_NOT_OPEN"),
    ("prediction_stable", "PREDICTION_NOT_STABLE"),
    ("servo_mapping_valid", "SERVO_MAPPING_INVALID"),
    ("servo_safety_valid", "SERVO_SAFETY_INVALID"),
    ("live_release_enabled", "LIVE_RELEASE_DISABLED"),
]


def evaluate_gates(gates: GateInputs, already_released: bool) -> List[str]:
    """Returns a list of block reasons. Empty list => all gates pass."""
    reasons = [reason for field_name, reason in _GATE_REASON_MAP if not getattr(gates, field_name)]
    if already_released:
        reasons.append("PAYLOAD_ALREADY_RELEASED")
    return reasons


@dataclass
class PayloadStateMachine:
    payload_id: int
    state: PayloadState = PayloadState.ARMED
    released: bool = False
    release_commanded: bool = False
    release_verified: bool = False
    last_block_reasons: List[str] = field(default_factory=list)

    def advance_to(self, target_state: PayloadState) -> None:
        """Advance forward only; never regress."""
        if _ORDER.index(target_state) > _ORDER.index(self.state):
            self.state = target_state

    def update(self, gates: GateInputs, waypoint_passed: bool) -> List[str]:
        """One state-machine tick. Progresses through the linear state
        chain as preconditions are satisfied, and evaluates the final
        release gate set. Returns the current block reasons (empty if
        release is allowed this cycle).
        """
        if self.released:
            self.state = PayloadState.DONE
            self.last_block_reasons = ["PAYLOAD_ALREADY_RELEASED"]
            return self.last_block_reasons

        if self.state == PayloadState.ARMED:
            self.advance_to(PayloadState.APPROACH)
        if self.state == PayloadState.APPROACH:
            self.advance_to(PayloadState.WAIT_WAYPOINT)
        if self.state == PayloadState.WAIT_WAYPOINT and waypoint_passed:
            self.advance_to(PayloadState.WAYPOINT_PASSED)
        if self.state == PayloadState.WAYPOINT_PASSED:
            self.advance_to(PayloadState.TARGET_VALIDATION)
        if self.state == PayloadState.TARGET_VALIDATION and gates.target_valid and gates.target_box_valid:
            self.advance_to(PayloadState.RELEASE_WINDOW)

        reasons = evaluate_gates(gates, self.released)
        self.last_block_reasons = reasons
        return reasons

    def commit_release(self) -> None:
        """Called exactly once by the servo controller after a verified
        release — never called speculatively.
        """
        self.released = True
        self.release_commanded = True
        self.state = PayloadState.RELEASE

    def mark_verified(self) -> None:
        self.release_verified = True
        self.state = PayloadState.DONE
