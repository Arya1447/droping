"""Servo command abstraction with mapping validation and one-shot latch.

Physics/prediction logic never talks to a MAVLink connection directly —
it goes through `ServoController`, which:

 1. validates the requested channel is actually mapped to the intended
    physical output (SERVOx_FUNCTION) before sending anything — real
    hardware mapping is UNKNOWN until read from the connected flight
    controller's parameters, so `MAVLinkServoController` defaults to
    servo_mapping_valid=False until a caller supplies a confirmed
    mapping (spec section 45/46/158);
 2. sends the release command exactly once per payload and then
    latches — no return-to-neutral, no repeated resend, no close
    command (spec sections 44/98/158);
 3. optionally verifies the commanded PWM against SERVO_OUTPUT_RAW /
    ACTUATOR_OUTPUT_STATUS feedback, when available.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Callable, Dict, Optional


class ServoFault(Exception):
    pass


@dataclass
class ServoCommandRecord:
    channel: int
    commanded_pwm: int
    actual_output: Optional[int] = None
    verification_status: str = "UNVERIFIED"  # UNVERIFIED / VERIFIED / MISMATCH / UNKNOWN


class ServoController:
    """Base abstraction. Subclasses implement `_send_pwm` and, if
    feedback is available, `_read_feedback`.
    """

    def __init__(self):
        self._latched: Dict[int, ServoCommandRecord] = {}

    def is_latched(self, channel: int) -> bool:
        return channel in self._latched

    def validate_mapping(self, channel: int) -> bool:
        """Must be overridden to check SERVOx_FUNCTION against the
        intended release function for `channel`. Base implementation
        conservatively returns False (UNKNOWN mapping => blocked),
        matching the spec's "never assume Servo N == index N" rule.
        """
        return False

    def release(self, channel: int, pwm: int) -> ServoCommandRecord:
        if self.is_latched(channel):
            # Already released — no duplicate command, ever.
            return self._latched[channel]

        if not self.validate_mapping(channel):
            raise ServoFault(f"servo channel {channel} mapping not validated; release blocked")

        self._send_pwm(channel, pwm)
        record = ServoCommandRecord(channel=channel, commanded_pwm=pwm)

        actual = self._read_feedback(channel)
        if actual is not None:
            record.actual_output = actual
            record.verification_status = "VERIFIED" if actual == pwm else "MISMATCH"
        else:
            record.verification_status = "UNKNOWN"

        self._latched[channel] = record
        return record

    def _send_pwm(self, channel: int, pwm: int) -> None:
        raise NotImplementedError

    def _read_feedback(self, channel: int) -> Optional[int]:
        return None


class FakeServoController(ServoController):
    """In-memory servo controller for tests — moves no real hardware."""

    def __init__(self, mapping_valid_channels: Optional[set] = None):
        super().__init__()
        self._mapping_valid_channels = mapping_valid_channels or set()
        self.sent_commands = []

    def set_mapping_valid(self, channel: int, valid: bool = True) -> None:
        if valid:
            self._mapping_valid_channels.add(channel)
        else:
            self._mapping_valid_channels.discard(channel)

    def validate_mapping(self, channel: int) -> bool:
        return channel in self._mapping_valid_channels

    def _send_pwm(self, channel: int, pwm: int) -> None:
        self.sent_commands.append((channel, pwm))

    def _read_feedback(self, channel: int) -> Optional[int]:
        if self.sent_commands and self.sent_commands[-1][0] == channel:
            return self.sent_commands[-1][1]
        return None


class MAVLinkServoController(ServoController):
    """Real servo controller over a MAVLink connection.

    `mapping_check` is an injected callable(channel) -> bool that must
    verify SERVOx_FUNCTION against the actual flight-controller
    parameter set (see mavlink_interface.py). Without a real connection
    supplying that check, mapping is treated as UNKNOWN/invalid and
    release stays blocked — this class never assumes servo N corresponds
    to output N.
    """

    def __init__(self, mavlink_conn, mapping_check: Optional[Callable[[int], bool]] = None):
        super().__init__()
        self._conn = mavlink_conn
        self._mapping_check = mapping_check

    def validate_mapping(self, channel: int) -> bool:
        if self._mapping_check is None:
            return False
        return bool(self._mapping_check(channel))

    def _send_pwm(self, channel: int, pwm: int) -> None:
        # MAV_CMD_DO_SET_SERVO — param1=servo channel, param2=PWM.
        # Verify command/param IDs against the pymavlink dialect actually
        # in use before deploying; this call assumes the common ArduPilot
        # mapping documented in the MAVLink common dialect.
        self._conn.mav.command_long_send(
            self._conn.target_system,
            self._conn.target_component,
            183,  # MAV_CMD_DO_SET_SERVO
            0,
            channel, pwm, 0, 0, 0, 0, 0,
        )

    def _read_feedback(self, channel: int) -> Optional[int]:
        # Left to caller: real feedback requires polling SERVO_OUTPUT_RAW
        # / ACTUATOR_OUTPUT_STATUS via mavlink_interface and mapping the
        # channel number to the right servoN_raw field. Returning None
        # here means verification_status will be UNKNOWN, not silently
        # assumed VERIFIED.
        return None
