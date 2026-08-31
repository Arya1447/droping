from telemetry_health import TelemetryHealth
import config


def _fully_fresh(th: TelemetryHealth, now: float, critical_only: bool = False):
    channels = ["gps", "altitude", "ground_speed", "airspeed", "heartbeat"]
    if not critical_only:
        channels += ["wind", "heading", "mission", "servo_feedback"]
    for ch in channels:
        th.touch(ch, True, now=now)


def test_healthy_when_all_critical_fresh():
    th = TelemetryHealth()
    now = 100.0
    for cycle in range(config.TELEMETRY_RECOVERY_CYCLES):
        _fully_fresh(th, now + cycle * 0.05)
    report = th.evaluate(now=now + config.TELEMETRY_RECOVERY_CYCLES * 0.05)
    assert report.state == "HEALTHY"
    assert report.critical_stale == []


def test_gps_stale_is_critical():
    th = TelemetryHealth()
    now = 100.0
    for cycle in range(config.TELEMETRY_RECOVERY_CYCLES):
        _fully_fresh(th, now + cycle * 0.05)
    later = now + config.TELEMETRY_RECOVERY_CYCLES * 0.05 + 5.0  # well past gps timeout
    report = th.evaluate(now=later)
    assert report.state == "CRITICAL"
    assert "gps" in report.critical_stale
    assert "GPS_STALE" in report.release_blocked_reasons()


def test_ground_speed_stale_is_critical():
    th = TelemetryHealth()
    now = 100.0
    for cycle in range(config.TELEMETRY_RECOVERY_CYCLES):
        _fully_fresh(th, now + cycle * 0.05)
    th.touch("ground_speed", True, now=now - 10.0)
    report = th.evaluate(now=now + config.TELEMETRY_RECOVERY_CYCLES * 0.05)
    assert report.state == "CRITICAL"
    assert "ground_speed" in report.critical_stale


def test_airspeed_stale_is_critical():
    th = TelemetryHealth()
    now = 100.0
    for cycle in range(config.TELEMETRY_RECOVERY_CYCLES):
        _fully_fresh(th, now + cycle * 0.05)
    th.touch("airspeed", True, now=now - 10.0)
    report = th.evaluate(now=now + config.TELEMETRY_RECOVERY_CYCLES * 0.05)
    assert report.state == "CRITICAL"
    assert "airspeed" in report.critical_stale


def test_wind_stale_is_warning_not_critical():
    th = TelemetryHealth()
    now = 100.0
    for cycle in range(config.TELEMETRY_RECOVERY_CYCLES):
        _fully_fresh(th, now + cycle * 0.05)
    th.touch("wind", True, now=now - 10.0)
    report = th.evaluate(now=now + config.TELEMETRY_RECOVERY_CYCLES * 0.05)
    assert report.state == "DEGRADED"
    assert "wind" in report.non_critical_stale
    assert report.critical_stale == []


def test_heartbeat_lost_is_critical():
    th = TelemetryHealth()
    now = 100.0
    for cycle in range(config.TELEMETRY_RECOVERY_CYCLES):
        _fully_fresh(th, now + cycle * 0.05)
    th.touch("heartbeat", True, now=now - 10.0)
    report = th.evaluate(now=now + config.TELEMETRY_RECOVERY_CYCLES * 0.05)
    assert report.state == "CRITICAL"
    assert "heartbeat" in report.critical_stale


def test_recovery_requires_consecutive_fresh_cycles():
    th = TelemetryHealth()
    now = 0.0
    # GPS starts stale (never touched) -> critical
    for ch in ["altitude", "ground_speed", "airspeed", "heartbeat"]:
        th.touch(ch, True, now=now)
    report = th.evaluate(now=now)
    assert report.state == "CRITICAL"

    # GPS becomes fresh for only 1 cycle -> should NOT immediately be trusted
    th.touch("gps", True, now=now)
    report2 = th.evaluate(now=now)
    assert "gps" in report2.critical_stale  # not yet recovered

    # Fresh for TELEMETRY_RECOVERY_CYCLES consecutive evaluations
    t = now
    for _ in range(config.TELEMETRY_RECOVERY_CYCLES):
        t += 0.05
        th.touch("gps", True, now=t)
        for ch in ["altitude", "ground_speed", "airspeed", "heartbeat"]:
            th.touch(ch, True, now=t)
        report_n = th.evaluate(now=t)
    assert "gps" not in report_n.critical_stale
