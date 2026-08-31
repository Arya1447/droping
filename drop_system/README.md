# drop_system — Autonomous Dual-Payload Drop System (KRTI, Fixed-Wing)

Physics-based, Monte-Carlo-validated, MAVLink/ArduPilot-integrated
autonomous payload release system for two independent payloads on a
fixed-wing UAV. Built to a strict anti-hallucination discipline: no
aerodynamic coefficient, sensor accuracy, or experimental result in
this codebase is invented. Every unavailable parameter is `None`
(UNKNOWN) and the corresponding code path fails closed rather than
guessing.

**This is a separate project from `/root/krti-flight-software`**, which
runs a different (vision-trigger) drop architecture. Nothing here
modifies that repository.

## 1. Installation

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

`pymavlink` is required for `DRY_RUN`/`LIVE` modes only. `pandas`,
`scikit-learn`, `matplotlib` are required for offline analysis
(`monte_carlo.py` via `analyze_results.py`, `calibration.py` via
`train_model.py`) — never imported by the live prediction/state-machine
path (spec section 118).

## 2. Configuration (`config.py`)

**Before any real flight or realistic simulation**, you must fill in:

- `TARGET_1["lat"]`, `TARGET_1["lon"]`, `TARGET_2["lat"]`, `TARGET_2["lon"]`
  — currently `None` (UNKNOWN). Until set, `target_valid` is False and
  release is blocked for both payloads (verified in `main.py --mode
  SIMULATION` smoke test — the system correctly refuses to release with
  placeholder coordinates rather than silently using (0, 0)).
- `LOCAL_ORIGIN_LAT` / `LOCAL_ORIGIN_LON` — local NEU frame origin.
- `TARGET_1["required_waypoint"]` / `TARGET_2["required_waypoint"]` —
  confirm the real mission sequence numbers (default: payload 1 = 6,
  per spec; payload 2 = 8 is a placeholder, confirm against the real
  mission).
- `MAVLINK_CONNECTION_STRING` — defaults to `udp:127.0.0.1:14551`,
  matching the existing `krti-flight-software` MAVProxy bridge
  convention; change if different.
- `ENABLE_LIVE_RELEASE` — must be explicitly set `True` by the operator
  for LIVE mode to ever command a servo. Defaults `False`.
- Drag model (`DRAG_ENABLED`, `DRAG_CD`, `DRAG_REFERENCE_AREA_M2`,
  payload geometry) — all `None`/disabled. Cd/reference area are
  genuinely UNKNOWN without wind-tunnel/CFD/measured data; do not fill
  these with guessed numbers.

Everything else (timeouts, filter alphas, stability thresholds,
acceptance policy) is a documented `CONFIGURED`/`ASSUMED`/`TEST` value
— see the comment on each constant in `config.py` for its provenance.

## 3. Mathematical model

See the MATHEMATICAL MODEL REVIEW delivered at the start of this
project's development (ballistic time-of-flight, Newton's-first-law
payload initial velocity, along/cross-track decomposition, mass-effect
analysis). Summary:

- **Ballistic (baseline):** `t_flight = (Vz0 + sqrt(Vz0^2 + 2*g*h)) / g`.
  Mass does **not** affect this (`a_gravity = F_gravity/m = g`),
  verified by `tests/test_physics.py::test_ballistic_mass_invariance_A_vs_B`.
- **Drag (optional, disabled by default):** `a_drag = F_drag/m`, mass
  **does** affect trajectory here — verified by
  `test_drag_mass_sensitivity_C_vs_D`. Only activates when
  `config.DRAG_CD`/`DRAG_REFERENCE_AREA_M2` are real, non-`None` values.
- **Payload initial velocity = aircraft ground velocity** at release
  (Newton's first law / translational inertia) — never zero.
- **Wind is a vector** (`V_wind = V_ground - V_air`, component-wise),
  never `ground_speed - airspeed` scalar subtraction (`wind_model.py`).

## 4. Coordinate system

Local North-East-Up (NEU) meters, equirectangular flat-earth
approximation around a configurable origin (`coordinate_utils.py`).
ASSUMED valid at KRTI mission scale (a few km); revisit for longer
ranges.

## 5. MAVLink mapping

See `mavlink_mapping.py` (`MAPPING_TABLE`) — declarative table of
message/field/unit/conversion/criticality for every telemetry variable
used, imported by the code (not just documentation). WIND/WIND_COV
message availability and the FROM/TO direction convention must be
verified against the actually-connected firmware before treating wind
data as MEASURED rather than ESTIMATED.

## 6. Telemetry, filtering, uncertainty

- `telemetry_health.py` — per-channel freshness, HEALTHY/DEGRADED/CRITICAL
  aggregation, N-cycle recovery gating for critical channels
  (`config.TELEMETRY_RECOVERY_CYCLES`).
- `filters.py` — EMA / circular EMA / median filters; smoothing vs
  latency trade-off documented in the module docstring.
- `uncertainty.py` — `UncertaintyEstimate` metadata (mean, sigma,
  sample_count, method, source, confidence) attached to every sigma
  used anywhere. Test-case sigmas are explicitly tagged
  ASSUMED/CONFIGURED, never presented as sensor specs.

## 7. Monte Carlo

`monte_carlo.py` fixes the release-trigger distance from nominal
`config.TEST_CASE` values, then samples only the genuinely uncertain
inputs (altitude, ground speed, heading, flight-path angle, wind,
servo delay, and payload mass under its documented ASSUMED sigma) —
target/aircraft GPS and target distance are never randomized (spec
section 63/80). Run:

```bash
python3 monte_carlo.py   # via analyze_results.py, see below
```

## 8. Calibration / empirical correction

`calibration.py` + `train_model.py` load a real flight-test CSV (schema
in spec section 97 / `calibration.REQUIRED_CSV_COLUMNS`), flag rows
VALID/SUSPICIOUS/INVALID (physical bounds + IQR), and cross-validate
four candidate models (physics-only / Ridge / degree-2 polynomial /
RandomForest) by out-of-fold MAE — never training error, never
auto-selecting the most complex model.

```bash
python3 train_model.py --csv <real_flight_test.csv> --out model_parameters.json
```

`example_drop_tests.csv` shipped here is **synthetic example data**
generated from this project's own Monte Carlo module purely to exercise
the pipeline — it is **not real flight-test data**, and metrics
produced from it must never be reported as real system performance.
`model_parameters.json` ships as `{"status": "NOT_TRAINED"}` until you
train on real data.

## 9. Target box vs. 5 m accuracy requirement

These are deliberately separate concepts (spec section 138):

- **Target box** (`config.TARGET_1["box"]`/`TARGET_2["box"]`) — an
  operational rectangular acceptance region you configure per target.
- **`config.MAX_ACCEPTABLE_IMPACT_ERROR_M = 5.0`** — the accuracy
  *performance requirement*, evaluated statistically (MAE, RMSE,
  MedianAE, STD, P95, Max, %-within-5m) over many predictions/trials,
  never used to alter physics constants.

`config.ACCEPTANCE_POLICY` (MAE≤5m AND P95≤5m AND within-5m%≥90%) is
fixed **before** looking at any results (spec section 136) — the 90%
threshold is itself an ASSUMED policy pending a competition-specified
rule; confirm with your team.

**Reported numbers must never be summarized as "system accuracy ≤5m"
unless MAE, P95, *and* Max all satisfy it** — see
`analyze_results.generate_acceptance_report()` for the exact language
used.

## 10. Waypoint validation

`waypoint_validator.py` uses segment-projection geometry (spec section
35/155), not `MISSION_CURRENT.seq >= required_waypoint` alone. Once
passed, latches permanently for that `WaypointValidator` instance.

## 11. Dual-payload state machine

`state_machine.py` — two fully independent `PayloadStateMachine`
instances (`ARMED → APPROACH → WAIT_WAYPOINT → WAYPOINT_PASSED →
TARGET_VALIDATION → RELEASE_WINDOW → RELEASE → DONE`). Releasing
payload 1 never touches payload 2's state.

## 12. Servo configuration

| Payload | Servo channel | Release PWM | Required waypoint |
|---|---|---|---|
| 1 | 7 (`config.TARGET_1["servo_channel"]`) | 2100 | 6 |
| 2 | 8 (`config.TARGET_2["servo_channel"]`) | 2100 | 8 (**confirm real value**) |

`servo_controller.py`: latch-once, no return-to-neutral, no repeated
resend. `MAVLinkServoController` requires an injected mapping-check
callback that verifies `SERVOx_FUNCTION` against real flight-controller
parameters — **without a real connection supplying that check, mapping
is UNKNOWN and release stays blocked**, by design.

## 13. Operating modes

```bash
python3 main.py --mode SIMULATION   # synthetic telemetry, no FC connection
python3 main.py --mode DRY_RUN      # real MAVLink telemetry, servo transport disabled (default)
python3 main.py --mode LIVE         # real servo commands (also requires config.ENABLE_LIVE_RELEASE=True)
```

Default mode is `DRY_RUN`. The `LIVE` telemetry-ingestion wiring
(mapping polled MAVLink messages into the prediction pipeline's input
dict) is scaffolded in `mavlink_interface.py`/`main.py` but the actual
message-to-field wiring against your specific flight controller stream
is left as an integration step — not fabricated here.

## 14. Safety

Every release gate (GPS/altitude/ground-speed/airspeed/heartbeat/EKF,
target/box/corridor, waypoint, prediction validity+stability, servo
mapping+safety, `ENABLE_LIVE_RELEASE`) must be `True` simultaneously;
a single failing gate produces an itemized `release_block_reason` list
— see `state_machine.evaluate_gates()`. Verified end-to-end in
`main.py --mode SIMULATION`: with placeholder target coordinates, the
system correctly reports `TARGET_INVALID` and blocks release rather
than proceeding with fabricated geometry.

## 15. Logging

`logger.py` writes one CSV row per prediction cycle per payload
(`config.LOG_CSV_PATH`, default `drop_system_log.csv`) plus a
companion `_actual_impact.csv` for post-flight measured impact data
(spec sections 82, 95-97, 113).

## 16. Deployment

Target: Linux/Armbian companion computer (matches the existing
`krti-flight-software` Raspberry Pi 5 / Orange Pi 5 setup). Live
runtime dependencies are numpy + pymavlink only; pandas/scikit-learn/
matplotlib stay offline-only (spec section 118).

## 17. Testing

```bash
python3 -m pytest tests/ -q
```

61 tests, covering ballistic fall time, mass invariance (ballistic) and
mass sensitivity (drag), wind vector decomposition, coordinate
transforms, waypoint spatial crossing + latch, telemetry freshness +
recovery + HEALTHY/DEGRADED/CRITICAL states, target box validation,
state-machine gating, servo latch/no-duplicate/mapping, uncertainty
estimation, Monte Carlo statistics, prediction stability, and an
end-to-end `predict_drop_point()` integration test. All 61 currently
pass. LIVE release is never exercised by automated tests (spec section
126).
