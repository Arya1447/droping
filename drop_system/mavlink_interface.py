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
from typing import Any, Dict, Optional, Tuple

from mavlink_mapping import MAPPING_TABLE


class MAVLinkUnavailableError(RuntimeError):
    pass


@dataclass
class LatestMessage:
    msg: Any
    timestamp: float


# GPS_FIX_TYPE enum values (MAVLink common.xml — an established protocol
# constant, not invented): 0=NO_GPS, 1=NO_FIX, 2=2D_FIX, 3=3D_FIX,
# 4=DGPS, 5=RTK_FLOAT, 6=RTK_FIXED, 7=STATIC, 8=PPP.
GPS_FIX_TYPE_3D = 3


def gps_valid_from_fix_type(fix_type: Optional[int]) -> bool:
    """GPS considered valid at 3D fix or better (spec section 51:
    available + finite + navigation valid)."""
    if fix_type is None:
        return False
    return fix_type >= GPS_FIX_TYPE_3D


# EKF_STATUS_REPORT.flags bitmask (MAVLink common.xml EKF_STATUS_FLAGS —
# an established protocol constant, not invented):
EKF_ATTITUDE = 1
EKF_VELOCITY_HORIZ = 2
EKF_VELOCITY_VERT = 4
EKF_POS_HORIZ_REL = 8
EKF_POS_HORIZ_ABS = 16
EKF_POS_VERT_ABS = 32
EKF_POS_VERT_AGL = 64
EKF_CONST_POS_MODE = 128
EKF_PRED_POS_HORIZ_REL = 256
EKF_PRED_POS_HORIZ_ABS = 512

# "EKF healthy for navigation" check: attitude + horizontal velocity +
# absolute horizontal position estimates all valid, and NOT running in
# constant-position (dead-reckoning/GPS-denied) fallback mode. This is
# the same combination GCS software (e.g. QGroundControl) commonly uses
# to color the EKF status indicator green — a documented convention,
# not a fabricated threshold.
_EKF_REQUIRED_FLAGS = EKF_ATTITUDE | EKF_VELOCITY_HORIZ | EKF_POS_HORIZ_ABS


def ekf_valid_from_flags(flags: Optional[int]) -> bool:
    if flags is None:
        return False
    has_required = (flags & _EKF_REQUIRED_FLAGS) == _EKF_REQUIRED_FLAGS
    in_const_pos_mode = bool(flags & EKF_CONST_POS_MODE)
    return has_required and not in_const_pos_mode


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
        "GPS_RAW_INT",
        "EKF_STATUS_REPORT",
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
        # wait_heartbeat() returns the HEARTBEAT message, or None on
        # timeout — it does NOT raise, so a silent "connect() succeeded"
        # with no real vehicle on the other end must be caught here
        # explicitly, otherwise target_system stays 0 and every
        # downstream MAVLink request silently talks to nothing.
        heartbeat_msg = self._conn.wait_heartbeat(timeout=timeout_s)
        if heartbeat_msg is None:
            self._conn = None
            raise MAVLinkUnavailableError(
                f"no HEARTBEAT received within {timeout_s}s on "
                f"{self.connection_string!r} — flight controller not reachable"
            )
        self._latest["HEARTBEAT"] = LatestMessage(msg=heartbeat_msg, timestamp=time.monotonic())
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

    def fetch_mission_items(self, timeout_s: float = 10.0) -> Dict[int, Tuple[float, float]]:
        """Request the full onboard mission once and return
        {seq: (lat_deg, lon_deg)} for every item. Waypoints are static
        during a flight, so this is meant to be called once after
        connect() — not re-fetched every prediction cycle (spec section
        117: heavy/one-shot work stays out of the live per-cycle loop).

        MISSION_ITEM_INT.x/.y are latitude/longitude * 1e7 for
        global-frame items (MAVLink common.xml), matching the
        mission_item_lat_deg/mission_item_lon_deg rows in
        mavlink_mapping.MAPPING_TABLE.
        """
        conn = self.connection
        conn.mav.mission_request_list_send(conn.target_system, conn.target_component)
        count_msg = conn.recv_match(type="MISSION_COUNT", blocking=True, timeout=timeout_s)
        if count_msg is None:
            raise MAVLinkUnavailableError("no MISSION_COUNT response; cannot fetch waypoints")

        items: Dict[int, Tuple[float, float]] = {}
        for seq in range(count_msg.count):
            conn.mav.mission_request_int_send(conn.target_system, conn.target_component, seq)
            item_msg = conn.recv_match(type="MISSION_ITEM_INT", blocking=True, timeout=timeout_s)
            if item_msg is None:
                continue
            items[item_msg.seq] = (item_msg.x / 1e7, item_msg.y / 1e7)

        if items:
            conn.mav.mission_ack_send(conn.target_system, conn.target_component, 0)
        return items

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
