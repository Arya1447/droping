"""Target and target-box validation (spec sections 29-32, 93, 138, 157).

A target box is a rectangular ACCEPTANCE/VALIDATION region in local
North/East coordinates, independent per target. It is deliberately NOT
the same thing as config.MAX_ACCEPTABLE_IMPACT_ERROR_M — the box is an
operational go/no-go gate the operator configures (e.g. a 5x5 m visible
drop zone), while the 5 m figure is a separate accuracy performance
requirement evaluated statistically over many predictions/trials. Do
not conflate the two: a box is not "radius = 5 m" by default.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Tuple


def target_valid(lat: Optional[float], lon: Optional[float]) -> bool:
    """A target is valid iff lat/lon are present, finite, and within
    plausible geodetic bounds. Does not verify EKF/navigation validity —
    that is a separate telemetry-health concern for the AIRCRAFT's own
    position, checked elsewhere.
    """
    if lat is None or lon is None:
        return False
    if not (math.isfinite(lat) and math.isfinite(lon)):
        return False
    if not (-90.0 <= lat <= 90.0):
        return False
    if not (-180.0 <= lon <= 180.0):
        return False
    return True


@dataclass
class TargetBox:
    north_m: float  # half-extent north of target center
    south_m: float  # half-extent south of target center
    east_m: float   # half-extent east of target center
    west_m: float   # half-extent west of target center

    def is_inside(self, delta_north_m: float, delta_east_m: float) -> bool:
        """delta_north_m/delta_east_m are the point's position relative
        to the target center in local coordinates.
        """
        return (
            -self.south_m <= delta_north_m <= self.north_m
            and -self.west_m <= delta_east_m <= self.east_m
        )


def is_inside_target_box(delta_north_m: float, delta_east_m: float, box: TargetBox) -> bool:
    return box.is_inside(delta_north_m, delta_east_m)


def cross_track_within_corridor(cross_track_error_m: float, max_cross_track_error_m: float) -> bool:
    """spec section 32: abs(cross_track_error) <= max_cross_track_error_m."""
    return abs(cross_track_error_m) <= max_cross_track_error_m


def target_ahead(target_along_track_m: float) -> bool:
    """spec section 25: target must be ahead of the aircraft (positive
    along-track distance) for a release computation to make sense.
    """
    return target_along_track_m > 0.0
