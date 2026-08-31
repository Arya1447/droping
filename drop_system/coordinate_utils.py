"""Coordinate transforms: lat/lon <-> local NEU, along/cross-track, bearing.

Local frame convention: North-East-Up (NEU), in meters, centered on a
configurable origin (config.LOCAL_ORIGIN_LAT/LON). An equirectangular
(flat-earth) approximation is used to convert between geodetic and
local coordinates:

    ASSUMED valid because KRTI-scale mission areas span at most a few
    kilometers, where WGS84 ellipsoid curvature error is negligible
    compared to the 5 m accuracy requirement's own error budget. This
    assumption would need revisiting for ranges beyond ~10 km.

Earth radius used: WGS84 mean radius, a well-established geodetic
constant (not an invented value).
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Tuple

EARTH_RADIUS_M = 6371000.0  # ASSUMED (WGS84 mean radius, standard constant)


def wrap_angle_deg(angle_deg: float) -> float:
    """Wrap an angle (or angle difference) into (-180, 180]."""
    return ((angle_deg + 180.0) % 360.0) - 180.0


def normalize_angle_deg(angle_deg: float) -> float:
    """Normalize an angle into [0, 360)."""
    return angle_deg % 360.0


def latlon_to_local(lat_deg: float, lon_deg: float,
                     origin_lat_deg: float, origin_lon_deg: float) -> Tuple[float, float]:
    """Convert geodetic (lat, lon) to local (North, East) meters relative
    to origin, using an equirectangular flat-earth approximation.
    """
    lat_rad = math.radians(lat_deg)
    origin_lat_rad = math.radians(origin_lat_deg)
    dlat_rad = math.radians(lat_deg - origin_lat_deg)
    dlon_rad = math.radians(lon_deg - origin_lon_deg)

    north_m = dlat_rad * EARTH_RADIUS_M
    east_m = dlon_rad * EARTH_RADIUS_M * math.cos(origin_lat_rad)
    return north_m, east_m


def local_to_latlon(north_m: float, east_m: float,
                     origin_lat_deg: float, origin_lon_deg: float) -> Tuple[float, float]:
    """Inverse of latlon_to_local."""
    origin_lat_rad = math.radians(origin_lat_deg)

    dlat_deg = math.degrees(north_m / EARTH_RADIUS_M)
    dlon_deg = math.degrees(east_m / (EARTH_RADIUS_M * math.cos(origin_lat_rad)))

    return origin_lat_deg + dlat_deg, origin_lon_deg + dlon_deg


def bearing_deg(delta_north_m: float, delta_east_m: float) -> float:
    """Bearing from origin to point (delta_N, delta_E), normalized [0,360)."""
    return normalize_angle_deg(math.degrees(math.atan2(delta_east_m, delta_north_m)))


def along_cross_track(delta_north_m: float, delta_east_m: float,
                       track_deg: float) -> Tuple[float, float]:
    """Decompose a (delta_N, delta_E) vector into along-track / cross-track
    components relative to a direction `track_deg` (e.g. ground track,
    heading, or bearing to waypoint).

    along = ΔN*cos(chi) + ΔE*sin(chi)
    cross = -ΔN*sin(chi) + ΔE*cos(chi)
    """
    chi = math.radians(track_deg)
    cos_c, sin_c = math.cos(chi), math.sin(chi)
    along = delta_north_m * cos_c + delta_east_m * sin_c
    cross = -delta_north_m * sin_c + delta_east_m * cos_c
    return along, cross


def horizontal_to_ne(v_horizontal: float, heading_deg: float) -> Tuple[float, float]:
    """Decompose a horizontal-plane speed magnitude + heading into N/E
    components: V_N = V_h*cos(psi), V_E = V_h*sin(psi).
    """
    psi = math.radians(heading_deg)
    return v_horizontal * math.cos(psi), v_horizontal * math.sin(psi)


def speed_gamma_to_vertical(v_magnitude: float, gamma_deg: float) -> Tuple[float, float]:
    """Decompose total speed + flight-path angle gamma into horizontal /
    vertical components: V_h = V*cos(gamma), V_v = V*sin(gamma).
    """
    gamma = math.radians(gamma_deg)
    return v_magnitude * math.cos(gamma), v_magnitude * math.sin(gamma)


def ground_track_from_velocity(v_north: float, v_east: float) -> float:
    """chi = atan2(V_E, V_N), normalized [0, 360)."""
    return normalize_angle_deg(math.degrees(math.atan2(v_east, v_north)))


def flight_path_angle_from_velocity(v_up: float, v_horizontal: float) -> float:
    """gamma = atan2(V_U, V_horizontal). +gamma = climbing, -gamma = descending."""
    return math.degrees(math.atan2(v_up, v_horizontal))


@dataclass
class Vector2D:
    north: float
    east: float

    def magnitude(self) -> float:
        return math.hypot(self.north, self.east)

    def __sub__(self, other: "Vector2D") -> "Vector2D":
        return Vector2D(self.north - other.north, self.east - other.east)

    def __add__(self, other: "Vector2D") -> "Vector2D":
        return Vector2D(self.north + other.north, self.east + other.east)
