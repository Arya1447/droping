import pytest

from servo_controller import FakeServoController, ServoFault
from state_machine import GateInputs, PayloadStateMachine, evaluate_gates


def _all_true_gates() -> GateInputs:
    return GateInputs(**{f: True for f in GateInputs.__dataclass_fields__})


def test_servo_release_and_latch():
    servo = FakeServoController(mapping_valid_channels={7})
    record = servo.release(7, 2100)
    assert record.commanded_pwm == 2100
    assert servo.is_latched(7)


def test_servo_no_duplicate_command():
    servo = FakeServoController(mapping_valid_channels={7})
    servo.release(7, 2100)
    servo.release(7, 2100)  # second call must not resend
    assert len(servo.sent_commands) == 1


def test_servo_mapping_invalid_blocks_release():
    servo = FakeServoController(mapping_valid_channels=set())  # channel 7 not mapped
    with pytest.raises(ServoFault):
        servo.release(7, 2100)


def test_payload1_and_payload2_independent_channels():
    servo = FakeServoController(mapping_valid_channels={7, 8})
    servo.release(7, 2100)
    assert servo.is_latched(7) is True
    assert servo.is_latched(8) is False
    servo.release(8, 2100)
    assert servo.is_latched(8) is True


def test_state_machine_all_gates_pass_allows_release():
    gates = _all_true_gates()
    reasons = evaluate_gates(gates, already_released=False)
    assert reasons == []


def test_state_machine_single_false_gate_blocks():
    gates = _all_true_gates()
    gates.gps_valid = False
    reasons = evaluate_gates(gates, already_released=False)
    assert "GPS_INVALID" in reasons


def test_state_machine_no_duplicate_release():
    sm = PayloadStateMachine(payload_id=1)
    sm.commit_release()
    gates = _all_true_gates()
    reasons = sm.update(gates, waypoint_passed=True)
    assert "PAYLOAD_ALREADY_RELEASED" in reasons


def test_payload_state_machines_are_independent():
    sm1 = PayloadStateMachine(payload_id=1)
    sm2 = PayloadStateMachine(payload_id=2)
    sm1.commit_release()
    assert sm1.released is True
    assert sm2.released is False
