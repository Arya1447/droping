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

## Quick start — how to actually run this

**Nothing here starts automatically.** `main.py` is a foreground
script you run by hand; there is no systemd service for it (unlike
`mavlink-supervisor.service`, which only brings up the MAVLink bridge,
not `drop_system` itself). Check `ps aux | grep main.py` if unsure
whether it's currently running — most of the time it won't be.

```bash
cd drop_system                      # imports are flat, must run from here
python3 -m venv .venv               # one-time setup
.venv/bin/pip install -r requirements.txt

# 1. Try it with zero hardware first:
.venv/bin/python main.py --mode SIMULATION --cycles 5

# 2. Once config.py is filled in (see "Configuration" below) AND
#    mavlink-supervisor.service is active (`systemctl is-active
#    mavlink-supervisor.service`) with real telemetry flowing on
#    udp:127.0.0.1:14556:
.venv/bin/python main.py --mode DRY_RUN            # no servo commands sent
.venv/bin/python main.py --mode LIVE               # ALSO needs config.ENABLE_LIVE_RELEASE = True
```

`--cycles N` runs N cycles then exits (useful for smoke-testing);
omit it to run forever until Ctrl+C. If `config.LOCAL_ORIGIN_LAT`/`LON`
are still `None`, `DRY_RUN`/`LIVE` no longer refuse to start — they
print a notice and sit on `HOLD` (release blocked) each cycle until
the aircraft's own first valid GPS fix (`fix_type>=3`) arrives, then
auto-capture that position as the origin for the rest of the run and
proceed normally. Manually setting `LOCAL_ORIGIN_LAT`/`LON` still
works and skips this wait entirely — manual always takes priority.

**What still needs filling in before this produces a real drop
decision** (all currently `None`/UNKNOWN placeholders in `config.py`):
`LOCAL_ORIGIN_LAT`/`LON`, `TARGET_1`/`TARGET_2` `lat`/`lon`, each
target's `geofence`, `SERVO_1`/`SERVO_2_EXPECTED_FUNCTION`. Until
those are set, every run correctly reports `TARGET_INVALID` /
`OUTSIDE_GEOFENCE` and blocks release — that is the fail-closed design
working as intended, not something broken.

**Known current caveat**: on this bench setup, `mavlink-supervisor.service`
restarts MAVProxy roughly every ~20s (traced to two `--out` targets in
`find_mavlink.py`'s `OUTS` pointing at Tailscale IPs while Tailscale is
logged out — `tailscale up` resolves it; left as-is for now by
operator choice). Expect `telemetry_health` to flip to `CRITICAL`
briefly during each restart — `DRY_RUN`/`LIVE` will correctly block
release during that window and recover on their own; this is not a
`drop_system` bug.

## 1. Installation

Run from inside `drop_system/` (imports here are flat — no package
`__init__.py` — so this must be the working directory both for the
venv and for every command below):

```bash
cd drop_system
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
  Optional to set by hand: if left `None`, `DRY_RUN`/`LIVE` auto-capture
  it from the aircraft's first valid GPS fix instead of refusing to
  start (see section 13).
- `TARGET_1["alt_m"]` / `TARGET_2["alt_m"]` — target ground elevation,
  same reference frame as `GLOBAL_POSITION_INT.relative_alt` (relative
  to the aircraft's home/launch point). `None` = ASSUMED level terrain
  (target at home's elevation); `prediction.py` subtracts this from
  `altitude_filtered_m` to get the actual ballistic drop height
  (`effective_drop_height_m`). Only leave `None` if the field is
  genuinely flat — otherwise survey it. If a target's elevation ends up
  at or above the aircraft's current altitude, release is blocked with
  `TARGET_ELEVATION_ABOVE_AIRCRAFT`, not silently miscalculated.
- `TARGET_1["required_waypoint"]` / `TARGET_2["required_waypoint"]` —
  confirm the real mission sequence numbers (default: payload 1 = 6,
  per spec; payload 2 = 8 is a placeholder, confirm against the real
  mission).
- `MAVLINK_CONNECTION_STRING` — defaults to `udp:127.0.0.1:14556`, a
  dedicated `--out` port added for this project in the sibling
  `krti-flight-software/find_mavlink.py`'s `OUTS` list (14551 is
  reserved for that supervisor's own heartbeat watchdog;
  14552-14555 already belong to `air_speed.py`/`batas_koordinat.py`/
  `ground_speed.py`/servo). Change if your setup differs.
- `ENABLE_LIVE_RELEASE` — must be explicitly set `True` by the operator
  for LIVE mode to ever command a servo. Defaults `False`.
- `SERVO_1_EXPECTED_FUNCTION` / `SERVO_2_EXPECTED_FUNCTION` — UNKNOWN
  (`None`) by default. `main.py` reads the real `SERVOx_FUNCTION`
  parameter from the connected flight controller and compares it
  against this value before trusting the servo mapping; leave `None`
  and release stays blocked until you check your vehicle's actual
  params and fill in the real function ID.
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

## 10a. Geofence

`geofence.py` is independent of, but follows the same convention as,
the sibling `krti-flight-software/batas_koordinat.py` — a rectangular/
polygonal boundary checked via point-in-polygon on raw lat/lon (plain
Python ray-casting here, no shapely dependency, no cross-project
import).

**Per-payload, not shared**: payload 1 and payload 2 are dropped in
different areas, so each target carries its own fence —
`TARGET_1["geofence"]` / `TARGET_2["geofence"]`, each a list of
`(lat, lon)` vertices. This is a different concept from
`batas_koordinat.py`'s single mission-wide `AREA` — don't assume the
two are the same polygon.

Both default to `None`/UNKNOWN — **fail-closed**: with no polygon
configured for that payload, its `geofence_valid` is always `False`
and release stays blocked, exactly like an unconfigured target. Every
prediction cycle logs `geofence_inside`/`geofence_source` and the live
debug output shows `Geofence: INSIDE/OUTSIDE (source=...)` per
payload.

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

Default mode is `DRY_RUN`. In `DRY_RUN`/`LIVE`, every cycle pulls fresh
data straight from MAVLink (`main.py::_live_telemetry`) — no hardcoded
telemetry constants in that path:

| Field | Source message.field | Notes |
|---|---|---|
| lat/lon/altitude/velocity N,E,U | `GLOBAL_POSITION_INT` | altitude via `relative_alt/1000` |
| heading | `GLOBAL_POSITION_INT.hdg` (falls back to `VFR_HUD.heading`) | `hdg==65535` treated as unknown per MAVLink spec |
| airspeed | `VFR_HUD.airspeed` | |
| wind | `WIND.speed`/`.direction` if streamed; else vector-estimated from `Vground - Vair` under an explicit ASSUMED-zero-sideslip label (`wind_source="ESTIMATED_ZERO_SIDESLIP_ASSUMED"`) | never silently falls back |
| GPS validity | `GPS_RAW_INT.fix_type >= 3` | MAVLink common.xml `GPS_FIX_TYPE` enum |
| EKF validity | `EKF_STATUS_REPORT.flags` (attitude+velocity+position bits set, not in const-pos-mode) | MAVLink common.xml `EKF_STATUS_FLAGS` |
| waypoint positions | `MISSION_ITEM_INT`, fetched once at startup via `MAVLinkInterface.fetch_mission_items()` | not re-fetched every cycle (spec section 117) |
| servo mapping | `SERVOx_FUNCTION` param, compared against `config.SERVO_1_EXPECTED_FUNCTION`/`SERVO_2_EXPECTED_FUNCTION` | **both `None`/UNKNOWN by default — you must fill these in** after checking your vehicle's real params, or servo release stays blocked |

`config.LOCAL_ORIGIN_LAT`/`LON` needs ONE fixed value across cycles
(local geometry breaks if it moves — waypoint spatial-crossing in
particular compares positions across time in the same frame). If left
`None`, `main()` no longer refuses to start: it prints a notice, sits
on `HOLD` each cycle (nothing computable without a frame — release
stays blocked), and auto-captures the aircraft's own position at its
first valid GPS fix (`fix_type>=3`, via `main._maybe_capture_origin`)
as the origin for the rest of that run. A manually-configured value
always takes priority and skips the wait. Either way, the origin is
purely a coordinate-math anchor — the aircraft-to-target distance that
actually matters for the drop decision is a delta, and is the same
regardless of which fixed point the origin happens to be (see
`coordinate_utils.latlon_to_local` — it cancels out algebraically);
home is used because it's the easiest position to obtain first and
because `relative_alt` is already home-relative, keeping N/E/U on one
consistent reference. `MAVLinkInterface.connect()` fails loudly
(`MAVLinkUnavailableError`) if no HEARTBEAT arrives within the connect
timeout, rather than silently proceeding with `target_system=0`.

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

83 tests, covering ballistic fall time, mass invariance (ballistic) and
mass sensitivity (drag), wind vector decomposition, coordinate
transforms, waypoint spatial crossing + latch, telemetry freshness +
recovery + HEALTHY/DEGRADED/CRITICAL states, target box validation,
state-machine gating, servo latch/no-duplicate/mapping, uncertainty
estimation, Monte Carlo statistics, prediction stability, target
elevation offset (level-terrain default, sloped-terrain correction,
target-above-aircraft fail-closed case), GPS/EKF validity thresholds,
geofence point-in-polygon + fail-closed-when-unconfigured, auto-origin
capture from first GPS fix, an end-to-end `predict_drop_point()`
integration test, and `test_live_mode_safety.py` (the durable version
of a manual real-FC verification — connected to the real flight
controller, fetched a real uploaded mission, ran full LIVE-mode cycles
for both payloads, zero servo commands sent while
`ENABLE_LIVE_RELEASE=False`). All 83 currently pass. Actual servo
firing (`ENABLE_LIVE_RELEASE=True`) is
never exercised by automated tests (spec section 126) — only the
guarantee that it stays off by default and blocks release regardless
of how favorable every other gate is.
