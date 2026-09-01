"""Verifies that `--mode LIVE` runs its full pipeline without crashing,
and — the actually load-bearing safety property — NEVER commands a
servo release while `config.ENABLE_LIVE_RELEASE` is `False` (the
shipped default), no matter how favorable every other gate is.

This is the durable, repeatable version of the manual real-MAVLink
verification run during development (connected to the real flight
controller, fetched a real uploaded mission, ran two full LIVE-mode
cycles for both payloads, exit_code=0, zero servo commands sent).
That one-off run proved connectivity works on real hardware; this
test proves the release-safety property holds structurally and stays
regression-tested going forward — it does not touch MAVLink at all
(spec section 126: LIVE release is never exercised by automated
tests), everything here runs through FakeServoController.
"""

import config
from logger import DropSystemLogger
from main import PayloadRuntime, run_cycle
from servo_controller import FakeServoController


def _all_gates_favorable_telemetry():
    """Telemetry chosen so every OTHER gate would pass: fresh/valid
    critical channels, waypoint already passed, inside the target box,
    inside the geofence, stable prediction. Only ENABLE_LIVE_RELEASE
    should be able to block release in this scenario.
    """
    return {
        "aircraft_lat": -6.9000, "aircraft_lon": 107.6000,
        "altitude_m": 20.0,
        "ground_velocity_n": 0.1, "ground_velocity_e": 0.0, "ground_velocity_u": 0.0,
        "air_speed_mps": 17.0, "heading_deg": 0.0,
        "wind_speed_mps": 0.0, "wind_direction_from_deg": 180.0,
        "wind_source": "MAVLINK_WIND_ESTIMATE", "wind_quality": "OK",
        "mission_seq": 99,
        "gps_valid": True, "ekf_valid": True, "heartbeat_valid": True,
        "ground_speed_valid": True, "airspeed_valid": True,
        "altitude_valid": True, "telemetry_health": "HEALTHY",
    }


def test_live_mode_never_releases_while_enable_live_release_is_false(tmp_path):
    assert config.ENABLE_LIVE_RELEASE is False, (
        "this test's whole premise is the shipped default being False; "
        "if this ever fires, the default itself changed and needs its "
        "own explicit review, not a silently-adjusted test"
    )

    target = dict(config.TARGET_1)
    target["geofence"] = [
        (-6.8990, 107.5990), (-6.8990, 107.6010),
        (-6.9010, 107.6010), (-6.9010, 107.5990),
    ]
    runtime = PayloadRuntime(payload_id=1, target=target)
    runtime.waypoint_prev_local = (0.0, 0.0)
    runtime.waypoint_current_local = (-10.0, 0.0)  # already behind the aircraft -> passed

    servo = FakeServoController(mapping_valid_channels={target["servo_channel"]})
    logger = DropSystemLogger(str(tmp_path / "test_log.csv"))

    telemetry = _all_gates_favorable_telemetry()

    ran = False
    for _ in range(6):  # a few cycles so the prediction-stability gate can settle
        ran = run_cycle(runtime, telemetry, "LIVE", servo, logger)

    assert ran is True
    assert servo.sent_commands == [], (
        "servo received a command while ENABLE_LIVE_RELEASE was False — "
        "this must never happen"
    )
    assert runtime.state_machine.released is False
    assert "LIVE_RELEASE_DISABLED" in runtime.state_machine.last_block_reasons


def test_live_mode_runs_full_cycle_without_crashing(tmp_path):
    """Structural smoke test: the LIVE code path (predict_drop_point,
    state machine, servo mapping check, logging) executes end-to-end
    for a full cycle without raising, even with several gates failing.
    """
    target = dict(config.TARGET_1)
    runtime = PayloadRuntime(payload_id=1, target=target)
    servo = FakeServoController()  # no channels mapped valid -> mapping gate fails, as it should
    logger = DropSystemLogger(str(tmp_path / "test_log2.csv"))

    telemetry = {
        "aircraft_lat": -6.9000, "aircraft_lon": 107.6000,
        "altitude_m": 20.0,
        "ground_velocity_n": 0.0, "ground_velocity_e": 0.0, "ground_velocity_u": 0.0,
        "gps_valid": False, "ekf_valid": False, "heartbeat_valid": False,
    }

    ran = run_cycle(runtime, telemetry, "LIVE", servo, logger)
    assert ran is True
    assert servo.sent_commands == []
