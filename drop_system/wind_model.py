"""Vector wind estimation and decomposition.

FINAL AUTHORITATIVE RULE (spec section 146): wind is a VECTOR quantity.
`wind_speed = ground_speed - airspeed` (scalar subtraction) is NEVER
used as the wind model here — it conflates two different reference
frames and direction is lost entirely.

Physics relation: V_ground = V_air + V_wind  =>  V_wind = V_ground - V_air,
evaluated component-wise in a common (North, East, Up) frame.

Wind direction convention: this module treats `wind_direction_deg` as
the FROM direction (meteorological convention: the compass direction
the wind is blowing FROM), matching MAVLink's WIND message semantics
(direction field is documented as "wind from" in ArduPilot's
usage) — VERIFY against the actual firmware/message definition in use
before trusting this on a real vehicle; if unconfirmed, treat the
convention as ASSUMED, not MEASURED.

To get the vector the wind is blowing TOWARD (used to add onto payload
velocity), rotate the FROM-direction by 180 degrees explicitly (see
`wind_vector_from_speed_direction`) — never done implicitly/silently.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Tuple

from coordinate_utils import normalize_angle_deg, along_cross_track


@dataclass
class WindEstimate:
    speed_mps: float
    direction_from_deg: float  # meteorological FROM convention
    source: str  # MAVLINK_WIND_ESTIMATE / ESTIMATED / UNKNOWN
    quality: str = "UNKNOWN"  # OK / DEGRADED / UNKNOWN
    sigma_speed_mps: Optional[float] = None
    sigma_direction_deg: Optional[float] = None
    uncertainty_source: str = "UNKNOWN"


def wind_vector_from_speed_direction(speed_mps: float, direction_from_deg: float) -> Tuple[float, float]:
    """Convert (speed, FROM-direction) into a (North, East) vector
    representing the direction air is moving TOWARD (i.e. the vector to
    add to an aircraft's airspeed vector to get ground velocity).

    The wind blows FROM `direction_from_deg`, so it travels TOWARD
    direction_from_deg + 180.
    """
    toward_deg = normalize_angle_deg(direction_from_deg + 180.0)
    theta = math.radians(toward_deg)
    return speed_mps * math.cos(theta), speed_mps * math.sin(theta)


def wind_speed_direction_from_vector(wind_n: float, wind_e: float) -> Tuple[float, float]:
    """Inverse: from a (North, East) TOWARD-vector, recover (speed,
    FROM-direction in degrees).
    """
    speed = math.hypot(wind_n, wind_e)
    if speed < 1e-6:
        # Direction undefined at near-zero wind speed (spec section 72).
        return speed, float("nan")
    toward_deg = normalize_angle_deg(math.degrees(math.atan2(wind_e, wind_n)))
    from_deg = normalize_angle_deg(toward_deg + 180.0)
    return speed, from_deg


def estimate_wind_vector(
    ground_n: float, ground_e: float, ground_u: float,
    air_n: Optional[float], air_e: Optional[float], air_u: Optional[float],
) -> Optional[Tuple[float, float, float]]:
    """V_wind = V_ground - V_air, component-wise. Returns None (UNKNOWN)
    if any air-relative component is unavailable (spec section 151:
    airspeed magnitude alone is not enough — a valid air-relative
    direction is required upstream to build air_n/air_e before calling
    this function).
    """
    if air_n is None or air_e is None or air_u is None:
        return None
    return ground_n - air_n, ground_e - air_e, ground_u - air_u


def wind_along_cross(wind_n: float, wind_e: float, track_deg: float) -> Tuple[float, float]:
    """Decompose wind vector into along-track / cross-track components
    relative to `track_deg` (e.g. ground track).
    """
    return along_cross_track(wind_n, wind_e, track_deg)


def consistency_residual(
    ground_n: float, ground_e: float, ground_u: float,
    air_n: float, air_e: float, air_u: float,
    wind_n: float, wind_e: float, wind_u: float,
) -> float:
    """residual = |V_ground - (V_air + V_wind)|. Large residual indicates
    sensor inconsistency (spec section 120/150) — used only as a
    WARNING/DEGRADED signal, never to force sensors to agree.
    """
    rn = ground_n - (air_n + wind_n)
    re = ground_e - (air_e + wind_e)
    ru = ground_u - (air_u + wind_u)
    return math.sqrt(rn * rn + re * re + ru * ru)


def propagate_wind_sigma_independent(sigma_ground: float, sigma_air: float) -> float:
    """sigma_wind^2 ~= sigma_ground^2 + sigma_air^2, valid ONLY when the
    ground and air velocity uncertainties are independent (spec section
    152). If a real covariance matrix between the two is available, use
    that instead — this function is the documented fallback, not a
    general-purpose propagation law.
    """
    return math.sqrt(sigma_ground ** 2 + sigma_air ** 2)
