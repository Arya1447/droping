"""Verifies run_cycle() no longer prints the full per-payload parameter
dump to stdout, and instead writes it to a live status file
(config.LIVE_LOG_FILE) when a path is given -- console output stays a
compact single-line status (built by main()'s loop itself, not
run_cycle()), so a running LIVE/DRY_RUN session doesn't spam dozens of
lines per cycle while every parameter is still visible in real time in
that file.
"""

import config
from logger import DropSystemLogger
from main import PayloadRuntime, run_cycle
from servo_controller import FakeServoController


def _telemetry():
    return {
        "aircraft_lat": -6.9000, "aircraft_lon": 107.6000,
        "altitude_m": 20.0,
        "ground_velocity_n": 0.0, "ground_velocity_e": 0.0, "ground_velocity_u": 0.0,
        "gps_valid": False, "ekf_valid": False, "heartbeat_valid": False,
    }


def test_run_cycle_does_not_print_detail_to_stdout(capsys, tmp_path):
    target = dict(config.TARGET_1)
    runtime = PayloadRuntime(payload_id=1, target=target)
    servo = FakeServoController()
    logger = DropSystemLogger(str(tmp_path / "log.csv"))

    run_cycle(runtime, _telemetry(), "LIVE", servo, logger)  # no live_log_path

    captured = capsys.readouterr()
    assert "=== PAYLOAD" not in captured.out
    assert "Airspeed" not in captured.out
    assert "RELEASE BLOCKED" not in captured.out


def test_run_cycle_writes_detail_to_live_log_file_when_given(tmp_path):
    target = dict(config.TARGET_1)
    runtime = PayloadRuntime(payload_id=1, target=target)
    servo = FakeServoController()
    logger = DropSystemLogger(str(tmp_path / "log.csv"))
    live_log = tmp_path / "droping.log"

    run_cycle(runtime, _telemetry(), "LIVE", servo, logger, live_log_path=str(live_log))

    assert live_log.exists()
    content = live_log.read_text()
    assert "=== PAYLOAD 1 ===" in content
    assert "Airspeed" in content
    assert "RELEASE BLOCKED" in content


def test_live_log_file_is_appended_within_one_cycle_not_overwritten_per_payload(tmp_path):
    """main.py truncates the file once per cycle (before the payload
    loop) then each run_cycle() call appends -- simulate two payloads
    writing to the same path and confirm both blocks survive.
    """
    servo = FakeServoController()
    logger = DropSystemLogger(str(tmp_path / "log.csv"))
    live_log = tmp_path / "droping.log"
    live_log.write_text("header\n\n")  # what main.py's per-cycle truncate+header does

    for pid, target_src in ((1, config.TARGET_1), (2, config.TARGET_2)):
        runtime = PayloadRuntime(payload_id=pid, target=dict(target_src))
        run_cycle(runtime, _telemetry(), "LIVE", servo, logger, live_log_path=str(live_log))

    content = live_log.read_text()
    assert "=== PAYLOAD 1 ===" in content
    assert "=== PAYLOAD 2 ===" in content
