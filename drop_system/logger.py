"""CSV logging for live/dry-run drop-system cycles (spec sections 82,
95-97, 113).

One row per prediction cycle, per payload, with the full field set
required by spec section 95 (dual-payload prefixed fields) plus
telemetry-health/freshness/state columns (section 113). A separate
`log_actual_impact()` appends post-flight measured impact data
(section 96) keyed by timestamp/payload so it can be joined back to
the prediction rows for calibration.py.
"""

from __future__ import annotations

import csv
import os
import time
from dataclasses import asdict
from typing import Dict, List, Optional

from prediction import PredictionResult

CYCLE_LOG_FIELDS = [
    "timestamp", "payload_id",
    "telemetry_health",
    "altitude_raw_m", "altitude_filtered_m", "altitude_source", "altitude_valid",
    "ground_speed_mps", "ground_track_deg", "air_speed_mps", "heading_deg", "flight_path_angle_deg",
    "wind_speed_mps", "wind_direction_from_deg", "wind_source", "wind_quality",
    "payload_mass_kg", "payload_mass_gram", "mass_source", "mass_uncertainty_kg",
    "predicted_fall_time_s", "predicted_drop_distance_m", "servo_displacement_m",
    "target_distance_m", "predicted_release_distance_m", "wind_correction_m",
    "empirical_correction_m", "final_release_distance_m",
    "predicted_impact_local_n", "predicted_impact_local_e",
    "predicted_impact_lat", "predicted_impact_lon",
    "signed_impact_error_along_m", "impact_error_cross_m", "predicted_impact_error_2d_m", "impact_status",
    "aircraft_cross_track_error_m",
    "gravity_force_n", "gravity_acceleration_mps2",
    "drag_force_n", "drag_acceleration_mps2", "ballistic_coefficient_kgm2",
    "target_valid", "target_box_valid", "predicted_impact_inside_box", "waypoint_valid",
    "prediction_stable", "prediction_uncertainty_m", "confidence",
    "release_allowed", "release_block_reason",
    "servo_output", "release_commanded", "release_verified",
]

ACTUAL_IMPACT_FIELDS = [
    "timestamp", "payload_id",
    "actual_impact_lat", "actual_impact_lon",
    "actual_impact_along_track", "actual_impact_cross_track",
    "actual_impact_error", "actual_impact_signed_error",
]


class DropSystemLogger:
    def __init__(self, csv_path: str, actual_impact_csv_path: Optional[str] = None):
        self.csv_path = csv_path
        self.actual_impact_csv_path = actual_impact_csv_path or (
            os.path.splitext(csv_path)[0] + "_actual_impact.csv"
        )
        self._ensure_header(self.csv_path, CYCLE_LOG_FIELDS)
        self._ensure_header(self.actual_impact_csv_path, ACTUAL_IMPACT_FIELDS)

    @staticmethod
    def _ensure_header(path: str, fields: List[str]) -> None:
        if not os.path.exists(path) or os.path.getsize(path) == 0:
            with open(path, "w", newline="") as f:
                writer = csv.DictWriter(f, fieldnames=fields)
                writer.writeheader()

    def log_cycle(self, result: PredictionResult, servo_output: Optional[int] = None,
                  release_commanded: bool = False, release_verified: bool = False,
                  timestamp: Optional[float] = None) -> None:
        row = asdict(result)
        row["timestamp"] = timestamp if timestamp is not None else time.time()
        row["release_block_reason"] = ";".join(result.release_block_reason)
        row["servo_output"] = servo_output
        row["release_commanded"] = release_commanded
        row["release_verified"] = release_verified

        out_row = {k: row.get(k) for k in CYCLE_LOG_FIELDS}
        with open(self.csv_path, "a", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=CYCLE_LOG_FIELDS)
            writer.writerow(out_row)

    def log_actual_impact(self, payload_id: int, actual_impact_lat: float, actual_impact_lon: float,
                           along_track_m: float, cross_track_m: float,
                           error_m: float, signed_error_m: float,
                           timestamp: Optional[float] = None) -> None:
        row = {
            "timestamp": timestamp if timestamp is not None else time.time(),
            "payload_id": payload_id,
            "actual_impact_lat": actual_impact_lat,
            "actual_impact_lon": actual_impact_lon,
            "actual_impact_along_track": along_track_m,
            "actual_impact_cross_track": cross_track_m,
            "actual_impact_error": error_m,
            "actual_impact_signed_error": signed_error_m,
        }
        with open(self.actual_impact_csv_path, "a", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=ACTUAL_IMPACT_FIELDS)
            writer.writerow(row)
