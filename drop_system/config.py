"""Central configuration for the autonomous dual-payload drop system.

Every numeric value here is tagged in the surrounding comment with its
provenance:

    CONFIGURED  - a value the user/operator sets deliberately (mission
                  geometry, servo channels, timeouts, policy thresholds).
    ASSUMED     - a physical constant assumed in the absence of a local
                  measurement (e.g. standard gravity).
    TEST        - values taken verbatim from the specification's "default
                  test case" section. Valid for SIMULATION/offline unit
                  tests and Monte Carlo nominal runs only. LIVE mode must
                  never use these in place of real telemetry.
    UNKNOWN     - genuinely unavailable data (Cd, reference area, real
                  servo latency, real sensor noise). Left as None; any
                  code path that needs it must degrade to a documented
                  UNKNOWN/blocked state rather than invent a number.

No aerodynamic coefficient, sensor accuracy, or experimental result in
this file is invented. Where the spec demanded a number that has no
authoritative source, the value is None and callers must treat that as
UNKNOWN.
"""

from __future__ import annotations

import dataclasses
from typing import Optional


# ============================================================
# OPERATING MODE
# ============================================================

# SIMULATION: no flight controller, synthetic telemetry.
# DRY_RUN:    live MAVLink telemetry + full prediction/state-machine,
#             servo command transport disabled.
# LIVE:       actual servo control enabled.
OPERATING_MODES = ("SIMULATION", "DRY_RUN", "LIVE")
DEFAULT_OPERATING_MODE = "DRY_RUN"  # CONFIGURED (spec section 99)

# Master safety switch. Even in LIVE mode, no servo command reaches
# hardware unless this is explicitly set True by the operator.
ENABLE_LIVE_RELEASE = False  # CONFIGURED, default per spec section 158/161


# ============================================================
# PHYSICS CONSTANTS
# ============================================================

GRAVITY_MPS2 = 9.81  # ASSUMED (standard gravity, not locally measured)

# Aerodynamic drag model. Disabled by default: Cd, reference area A and
# air density rho for the actual payload are UNKNOWN (no wind-tunnel,
# CFD, or measured data supplied). Do not invent these.
DRAG_ENABLED = False
DRAG_CD: Optional[float] = None          # UNKNOWN
DRAG_REFERENCE_AREA_M2: Optional[float] = None  # UNKNOWN
AIR_DENSITY_KGM3: Optional[float] = 1.225  # ASSUMED ISA sea-level; UNKNOWN if site density differs materially

# Payload geometry (cylinder + conical nose). None = UNKNOWN; fill in
# only with measured payload dimensions.
PAYLOAD_DIAMETER_M: Optional[float] = None       # UNKNOWN
PAYLOAD_CYLINDER_LENGTH_M: Optional[float] = None  # UNKNOWN
PAYLOAD_NOSE_LENGTH_M: Optional[float] = None      # UNKNOWN


# ============================================================
# PAYLOAD MASS (spec section 17A / 153)
# ============================================================

PAYLOAD_MASS_KG_NOMINAL = 0.500  # CONFIGURED nominal, per spec section 3
PAYLOAD_MASS_SOURCE = "CONFIGURED"  # not MEASURED until real weighings logged
# Mass uncertainty: no weighing data supplied. Test Monte Carlo sigma is
# a TEST ASSUMPTION (section 3), not a claim about actual payload-to-
# payload variation.
PAYLOAD_MASS_SIGMA_KG_TEST = 0.005  # TEST
PAYLOAD_MASS_SIGMA_SOURCE = "ASSUMED"


# ============================================================
# DEFAULT TEST CASE (spec section 3) — offline/simulation/unit-test only
# ============================================================

@dataclasses.dataclass(frozen=True)
class TestCase:
    payload_mass_kg: float = 0.5
    wind_speed_mps: float = 5.0
    altitude_m: float = 100.0
    ground_speed_mps: float = 18.0
    air_speed_mps: float = 17.0
    heading_deg: float = 0.0
    flight_path_angle_deg: float = 0.0
    servo_delay_s: float = 0.30
    monte_carlo_trials: int = 10_000


TEST_CASE = TestCase()


@dataclasses.dataclass(frozen=True)
class TestUncertainty:
    altitude_sigma_m: float = 1.0
    ground_speed_sigma_mps: float = 0.3
    air_speed_sigma_mps: float = 0.3
    heading_sigma_deg: float = 1.0
    flight_path_angle_sigma_deg: float = 0.3
    wind_speed_sigma_mps: float = 0.5
    wind_direction_sigma_deg: float = 5.0
    payload_mass_sigma_kg: float = 0.005
    servo_delay_sigma_s: float = 0.02


TEST_UNCERTAINTY = TestUncertainty()


# ============================================================
# SERVO DELAY
# ============================================================

SERVO_DELAY_S_TEST = 0.30  # TEST default (section 3); real latency UNKNOWN
SERVO_DELAY_SOURCE = "CONFIGURED"


# ============================================================
# PERFORMANCE REQUIREMENT (spec section 144, final/authoritative)
# ============================================================

MAX_ACCEPTABLE_IMPACT_ERROR_M = 5.0  # CONFIGURED performance requirement,
# NOT a physics constant. Never used to alter g, mass, Cd, wind, etc.

# Acceptance policy, fixed BEFORE looking at any validation results
# (spec section 136). This is itself an ASSUMED policy in the absence of
# a competition-specified acceptance rule; document/confirm with the
# team before relying on PASS/FAIL output for a real decision.
ACCEPTANCE_POLICY = {
    "require_mae_le": 5.0,
    "require_p95_le": 5.0,
    "require_within_5m_pct_ge": 90.0,  # ASSUMED threshold
    "min_samples_for_verdict": 30,  # below this -> INSUFFICIENT_DATA
}


# ============================================================
# TELEMETRY TIMEOUTS (spec section 54) — TEST DEFAULTS
# ============================================================

GPS_TIMEOUT_S = 0.5           # TEST default
ALTITUDE_TIMEOUT_S = 0.5      # TEST default
GROUND_SPEED_TIMEOUT_S = 0.5  # TEST default
AIRSPEED_TIMEOUT_S = 0.5      # TEST default
HEARTBEAT_TIMEOUT_S = 1.0     # TEST default
WIND_TIMEOUT_S = 2.0          # TEST default (non-critical)
HEADING_TIMEOUT_S = 1.0       # TEST default (non-critical)
MISSION_TIMEOUT_S = 2.0       # TEST default (non-critical)
SERVO_FEEDBACK_TIMEOUT_S = 2.0  # TEST default (non-critical)

TELEMETRY_RECOVERY_CYCLES = 3  # CONFIGURED (spec section 59)


# ============================================================
# FILTERING (spec section 62)
# ============================================================

EMA_ALPHA_GROUND_SPEED = 0.3   # CONFIGURED
EMA_ALPHA_AIR_SPEED = 0.3      # CONFIGURED
EMA_ALPHA_ALTITUDE = 0.3       # CONFIGURED
EMA_ALPHA_HEADING = 0.3        # CONFIGURED (circular EMA)
EMA_ALPHA_WIND = 0.2           # CONFIGURED
MEDIAN_FILTER_WINDOW = 3        # CONFIGURED, kept small to limit latency


# ============================================================
# PREDICTION STABILITY (spec section 42/92)
# ============================================================

PREDICTION_STABLE_CYCLES = 5          # TEST default
MAX_PREDICTION_CHANGE_M = 1.0         # TEST default


# ============================================================
# TARGETS / PAYLOADS (spec sections 28, 29, 90)
# ============================================================
#
# lat/lon are UNKNOWN placeholders (None). The operator MUST fill in
# real mission target coordinates before LIVE or realistic SIMULATION
# use. Any code path that receives lat=None/lon=None must report
# target_valid=False, never silently default to (0,0).

TARGET_1 = {
    "lat": 0.0000,  # harus diisi sendiri
    "lon": 0.0000,  # harus diisi sendiri
    # Target ground elevation, in the SAME reference frame as
    # GLOBAL_POSITION_INT.relative_alt (i.e. relative to the aircraft's
    # home/launch point, positive = higher than home). Used by
    # prediction.py to correct the ballistic drop height when the
    # target's terrain elevation differs from home's. None = ASSUMED
    # level terrain (target at the same elevation as home) — only safe
    # to leave None if the field is genuinely flat; otherwise survey it
    # (e.g. GPS altitude at home minus GPS altitude at target).
    "alt_m": None,
    "box": {
        "north_m": 3.0,  # CONFIGURED acceptance region half-extents
        "south_m": 3.0,
        "east_m": 3.0,
        "west_m": 3.0,
    },
    "required_waypoint": 6,  # CONFIGURED, per spec section 33/156
    "servo_channel": 6,      # CONFIGURED
    "release_pwm": 2100,     # CONFIGURED
    "max_cross_track_error_m": 7.0,  # CONFIGURED flight corridor
    # Geofence for THIS payload's drop run only (payload 1 and payload 2
    # use different areas — do not share one polygon between them).
    # List of (lat, lon) vertices, same point-in-polygon convention as
    # krti-flight-software/batas_koordinat.py's AREA (that file's own
    # polygon is a single shared mission-wide fence, a different concept
    # from this per-payload drop-zone fence — don't assume they're the
    # same points).
    #
    # !!! PLACEHOLDER / EXAMPLE ONLY !!!
    # These four points are NOT a real survey and are NOT tied to any
    # actual mission location — deliberately round numbers (4 decimal
    # places) so they cannot be mistaken for real GPS survey data (which
    # typically has 6+ decimal places, e.g. batas_koordinat.py's AREA).
    # They exist only so geofence_valid has something non-None to
    # evaluate while the pipeline is being developed/tested. Source =
    # "CONFIGURED" in logs will look identical to a real fence — REPLACE
    # these with a real surveyed/committee-provided boundary for payload
    # 1's drop zone before any DRY_RUN/LIVE use that matters.
    # Order convention: utara kiri -> utara kanan -> selatan kanan ->
    # selatan kiri (same clockwise order as batas_koordinat.py's AREA).
    "geofence": [
        (-6.9000, 107.6000),  # utara kiri    lat, lon
        (-6.9000, 107.6009),  # utara kanan   lat, lon
        (-6.9009, 107.6009),  # selatan kanan lat, lon
        (-6.9009, 107.6000),  # selatan kiri  lat, lon
    ],
}

TARGET_2 = {
    "lat": 0.0000,  # UNKNOWN - fill with real mission target 2 latitude
    "lon": 0.0000,  # UNKNOWN - fill with real mission target 2 longitude
    "alt_m": None,  # see TARGET_1["alt_m"] comment — same semantics
    "box": {
        "north_m": 3.0,
        "south_m": 3.0,
        "east_m": 3.0,
        "west_m": 3.0,
    },
    "required_waypoint": 8,  # CONFIGURED placeholder - confirm real mission seq
    "servo_channel": 7,      # CONFIGURED
    "release_pwm": 2100,     # CONFIGURED
    "max_cross_track_error_m": 10.0,
    # !!! PLACEHOLDER / EXAMPLE ONLY !!! — see TARGET_1["geofence"]
    # comment above; same caveats apply. Deliberately a DIFFERENT
    # example area from payload 1's (not because of any real mission
    # requirement — just to keep the two payloads' placeholders visibly
    # distinct while both are still fake). Replace with payload 2's real
    # surveyed drop-zone boundary before any use that matters.
    # Order convention: utara kiri -> utara kanan -> selatan kanan ->
    # selatan kiri (same clockwise order as batas_koordinat.py's AREA).
    "geofence": [
        (-6.9020, 107.6020),  # utara kiri    lat, lon
        (-6.9020, 107.6029),  # utara kanan   lat, lon
        (-6.9029, 107.6029),  # selatan kanan lat, lon
        (-6.9029, 107.6020),  # selatan kiri  lat, lon
    ],
}

TARGETS = {1: TARGET_1, 2: TARGET_2}


# ============================================================
# LOCAL COORDINATE ORIGIN
# ============================================================
#
# Local NEU frame origin for lat/lon <-> local conversions. UNKNOWN
# until set to a real mission reference point (e.g. home position or
# target 1). Equirectangular flat-earth approximation is used
# (ASSUMED valid at KRTI-scale distances, a few km at most).
LOCAL_ORIGIN_LAT: Optional[float] = None  # UNKNOWN
LOCAL_ORIGIN_LON: Optional[float] = None  # UNKNOWN


# ============================================================
# MAVLINK CONNECTION
# ============================================================

MAVLINK_CONNECTION_STRING = "udp:127.0.0.1:14556"  # CONFIGURED, dedicated
# --out port added for drop_system in krti-flight-software/find_mavlink.py's
# OUTS list (14551 is reserved for that supervisor's own heartbeat
# watchdog; 14552/14553/14554/14555 already belong to
# air_speed.py/batas_koordinat.py/ground_speed.py/servo).
MAVLINK_SOURCE_SYSTEM = 255  # CONFIGURED (ground-station-style GCS id)

# Expected SERVOx_FUNCTION parameter value for each payload's release
# channel — this is what main.py compares the REAL flight-controller
# param against before trusting the mapping (spec section 45/158: never
# assume Servo N == output N). UNKNOWN until you check the actual
# vehicle's params (e.g. via Mission Planner/QGC parameter list, or
# `mavlink.get_servo_function_param(7)` after connecting) and confirm
# what SERVO7_FUNCTION/SERVO8_FUNCTION are actually set to (commonly a
# Relay or Script/RCIN passthrough function for a payload-release
# mechanism — ArduPilot has no single dedicated "payload release"
# SERVO_FUNCTION enum value, so this genuinely depends on your specific
# gripper/servo wiring). Leaving these None keeps servo_mapping_valid
# False (release blocked) until you fill in the real number.
SERVO_1_EXPECTED_FUNCTION: Optional[int] = None  # UNKNOWN
SERVO_2_EXPECTED_FUNCTION: Optional[int] = None  # UNKNOWN


# ============================================================
# LIVE LOOP TIMING
# ============================================================

PREDICTION_CYCLE_HZ = 10.0  # CONFIGURED target live-loop rate — raw
# GLOBAL_POSITION_INT arrives ~19 Hz (measured), so 10 Hz consumption
# still always gets fresh (<=100ms old) data, never starved.


# ============================================================
# LOGGING
# ============================================================

LOG_CSV_PATH = "drop_system_log.csv"  # CONFIGURED

# Live human-readable status file (spec-external addition, not a spec
# section): overwritten fresh at the start of every prediction cycle,
# then each payload appends its block — so `watch cat droping.log` (or
# any editor with auto-reload) shows current numbers. The console
# itself only ever prints one line for the whole run (see main.py).
#
# Deliberately placed on /dev/shm (RAM-backed tmpfs), not on disk/SD
# card/eMMC: this file is rewritten every prediction cycle (10x/sec by
# default) for as long as the program runs -- on a real block device
# that's a meaningful, pointless write-wear/IO cost for data that's
# only ever meant to be glanced at live and is deleted the moment the
# program exits anyway (see the `finally` block in main()). tmpfs
# means it never touches persistent storage at all, at the cost of a
# few KB of RAM only while the program is running -- freed automatically
# on exit along with the file itself. /dev/shm is standard on virtually
# every Linux system (verified present on this one: `mount | grep shm`);
# if it's ever missing on a target machine, writes here just silently
# fail (main.py's open() calls already swallow OSError, matching the
# "no console fallback" design) rather than falling back to disk --
# change this path explicitly if that ever needs to be different.
LIVE_LOG_FILE = "/dev/shm/droping.log"  # CONFIGURED
