"""Thin MAVLink connection wrapper.

pymavlink is only required for LIVE/DRY_RUN modes (spec section 118:
minimal dependencies, heavy libs kept offline). Import is deferred so
this module can be imported (and its pure functions tested) even on a
machine without pymavlink installed; only `MAVLinkInterface.connect()`
requires the real dependency.

Every getter here returns raw message data plus a timestamp; freshness
and criticality logic live in telemetry_health.py, not here — this
module's only job is "get the latest message and when it arrived".
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Any, Dict, Optional

from mavlink_mapping import MAPPING_TABLE


class MAVLinkUnavailableError(RuntimeError):
    pass


@dataclass
class LatestMessage:
    msg: Any
    timestamp: float


class MAVLinkInterface:
    """Wraps a pymavlink connection, tracking the most recent instance of
    each message type of interest by name.
    """

    MESSAGE_TYPES = (
        "HEARTBEAT",
        "GLOBAL_POSITION_INT",
        "VFR_HUD",
        "WIND",
        "WIND_COV",
        "MISSION_CURRENT",
        "MISSION_ITEM_INT",
        "SERVO_OUTPUT_RAW",
        "ACTUATOR_OUTPUT_STATUS",
    )

    def __init__(self, connection_string: str, source_system: int = 255):
        self.connection_string = connection_string
        self.source_system = source_system
        self._conn = None
        self._latest: Dict[str, LatestMessage] = {}

    def connect(self, timeout_s: float = 10.0):
        try:
            from pymavlink import mavutil
        except ImportError as exc:
            raise MAVLinkUnavailableError(
                "pymavlink is not installed in this environment; "
                "required for LIVE/DRY_RUN telemetry."
            ) from exc

        self._conn = mavutil.mavlink_connection(
            self.connection_string, source_system=self.source_system
        )
        self._conn.wait_heartbeat(timeout=timeout_s)
        self._latest["HEARTBEAT"] = LatestMessage(msg=None, timestamp=time.monotonic())
        return self._conn

    @property
    def connection(self):
        if self._conn is None:
            raise MAVLinkUnavailableError("not connected; call connect() first")
        return self._conn

    def poll(self, blocking: bool = False) -> Optional[str]:
        """Read one available message (non-blocking by default) and
        record it if it's a type of interest. Returns the message type
        name, or None if nothing was available.
        """
        msg = self.connection.recv_match(blocking=blocking)
        if msg is None:
            return None
        msg_type = msg.get_type()
        if msg_type in self.MESSAGE_TYPES:
            self._latest[msg_type] = LatestMessage(msg=msg, timestamp=time.monotonic())
        return msg_type

    def get_latest(self, message_type: str) -> Optional[LatestMessage]:
        return self._latest.get(message_type)

    def get_servo_function_param(self, channel: int) -> Optional[int]:
        """Read SERVOx_FUNCTION from flight-controller parameters, used
        by servo_controller.MAVLinkServoController's mapping_check.
        Returns None (UNKNOWN) if the parameter hasn't been fetched yet
        — caller must not assume a default.
        """
        param_name = f"SERVO{channel}_FUNCTION"
        try:
            value = self.connection.params.get(param_name)
        except AttributeError:
            return None
        return int(value) if value is not None else None
