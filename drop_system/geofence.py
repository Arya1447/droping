"""Geofence validation — independent of, but same convention as, the
existing `krti-flight-software/batas_koordinat.py` (rectangular/
polygonal boundary check via point-in-polygon on raw lat/lon, no
projection — matching that script's own approach of feeding
(lon, lat) straight into shapely without reprojecting).

Kept self-contained in `drop_system` (no shapely dependency, no
cross-project import) via a plain-Python ray-casting point-in-polygon
test, so this module needs nothing beyond the standard library.

config.GEOFENCE_POLYGON is UNKNOWN (None) by default. If your mission
uses the same boundary as `batas_koordinat.py`'s `AREA` list, copy
those exact (lat, lon) points here — the two files are independently
maintained, so keeping them in sync is the operator's responsibility,
not automated.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional, Tuple


@dataclass
class GeofenceStatus:
    inside: bool
    polygon_configured: bool
    source: str  # CONFIGURED / UNKNOWN


def point_in_polygon(lat: float, lon: float, polygon: List[Tuple[float, float]]) -> bool:
    """Ray-casting point-in-polygon test on raw (lat, lon) pairs, treated
    as planar (x, y) coordinates — valid at the same local scale this
    entire project already assumes (a few km at most; see
    coordinate_utils.py's flat-earth approximation note). `polygon` is a
    list of (lat, lon) vertices; does not need to repeat the first point
    at the end.
    """
    n = len(polygon)
    if n < 3:
        return False

    inside = False
    x, y = lon, lat  # treat lon as x, lat as y for the standard algorithm
    x1, y1 = polygon[-1][1], polygon[-1][0]
    for lat_i, lon_i in polygon:
        x2, y2 = lon_i, lat_i
        if ((y1 > y) != (y2 > y)) and (x < (x2 - x1) * (y - y1) / (y2 - y1) + x1):
            inside = not inside
        x1, y1 = x2, y2
    return inside


def check_geofence(lat: Optional[float], lon: Optional[float],
                    polygon: Optional[List[Tuple[float, float]]]) -> GeofenceStatus:
    """Fail-closed: if the polygon isn't configured, or lat/lon are
    missing, `inside=False` — never assume "no fence configured" means
    "anywhere is fine" for a safety-relevant gate.
    """
    if polygon is None or len(polygon) < 3:
        return GeofenceStatus(inside=False, polygon_configured=False, source="UNKNOWN")
    if lat is None or lon is None:
        return GeofenceStatus(inside=False, polygon_configured=True, source="CONFIGURED")
    return GeofenceStatus(
        inside=point_in_polygon(lat, lon, polygon),
        polygon_configured=True,
        source="CONFIGURED",
    )
