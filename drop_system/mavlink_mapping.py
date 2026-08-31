"""Declarative MAVLink message->variable mapping table (spec section 50).

This is data, not just documentation: mavlink_interface.py imports
`MAPPING_TABLE` to know which message/field to read and how to convert
units, so the mapping can't silently drift out of sync with the code
that uses it.

Every row's `criticality` matches telemetry_health.CRITICAL_CHANNELS /
NON_CRITICAL_CHANNELS. `fallback` is None where no safe fallback exists
(per spec section 116: no silent fallback) — in that case the caller
must report the source as UNAVAILABLE, not substitute a guess.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, Optional


@dataclass(frozen=True)
class MappingEntry:
    variable: str
    message: str
    field: str
    raw_unit: str
    converted_unit: str
    conversion: Callable[[float], float]
    update_rate_hz: Optional[float]  # UNKNOWN unless measured on the real link
    criticality: str  # CRITICAL / NON_CRITICAL
    fallback: Optional[str]


MAPPING_TABLE = [
    MappingEntry("latitude_deg", "GLOBAL_POSITION_INT", "lat", "1e-7 deg", "deg",
                 lambda v: v / 1e7, None, "CRITICAL", None),
    MappingEntry("longitude_deg", "GLOBAL_POSITION_INT", "lon", "1e-7 deg", "deg",
                 lambda v: v / 1e7, None, "CRITICAL", None),
    MappingEntry("altitude_m", "GLOBAL_POSITION_INT", "relative_alt", "mm", "m",
                 lambda v: v / 1000.0, None, "CRITICAL", None),
    MappingEntry("velocity_north_mps", "GLOBAL_POSITION_INT", "vx", "cm/s", "m/s",
                 lambda v: v / 100.0, None, "CRITICAL", None),
    MappingEntry("velocity_east_mps", "GLOBAL_POSITION_INT", "vy", "cm/s", "m/s",
                 lambda v: v / 100.0, None, "CRITICAL", None),
    MappingEntry("velocity_up_mps", "GLOBAL_POSITION_INT", "vz", "cm/s (down-positive)", "m/s (up-positive)",
                 lambda v: -v / 100.0, None, "CRITICAL", None),
    MappingEntry("heading_deg", "GLOBAL_POSITION_INT", "hdg", "cdeg", "deg",
                 lambda v: v / 100.0, None, "NON_CRITICAL", "VFR_HUD.heading"),
    MappingEntry("airspeed_mps", "VFR_HUD", "airspeed", "m/s", "m/s",
                 lambda v: v, None, "CRITICAL", None),
    MappingEntry("groundspeed_mps", "VFR_HUD", "groundspeed", "m/s", "m/s",
                 lambda v: v, None, "NON_CRITICAL", "sqrt(vx^2+vy^2) from GLOBAL_POSITION_INT"),
    MappingEntry("wind_speed_mps", "WIND", "speed", "m/s", "m/s",
                 lambda v: v, None, "NON_CRITICAL", "estimated from Vground-Vair (section 146/147)"),
    MappingEntry("wind_direction_from_deg", "WIND", "direction", "deg (FROM, unverified)", "deg",
                 lambda v: v, None, "NON_CRITICAL", "estimated from Vground-Vair"),
    MappingEntry("mission_current_seq", "MISSION_CURRENT", "seq", "int", "int",
                 lambda v: v, None, "CRITICAL", None),
    MappingEntry("mission_item_lat_deg", "MISSION_ITEM_INT", "x", "1e-7 deg", "deg",
                 lambda v: v / 1e7, None, "CRITICAL", None),
    MappingEntry("mission_item_lon_deg", "MISSION_ITEM_INT", "y", "1e-7 deg", "deg",
                 lambda v: v / 1e7, None, "CRITICAL", None),
    MappingEntry("servo_output_us", "SERVO_OUTPUT_RAW", "servoN_raw", "us", "us",
                 lambda v: v, None, "NON_CRITICAL", "ACTUATOR_OUTPUT_STATUS.actuator[N]"),
    MappingEntry("heartbeat_present", "HEARTBEAT", "(presence)", "-", "-",
                 lambda v: v, None, "CRITICAL", None),
]

# Note: WIND message field-name/unit definitions (and whether it or
# WIND_COV is even streamed) must be verified against the connected
# ArduPilot firmware version before trusting the wind_speed_mps /
# wind_direction_from_deg rows above as MEASURED — until then, treat
# any wind value obtained this way as ESTIMATED at best, and confirm
# the FROM-vs-TO direction convention against real telemetry (spec
# section 148).
