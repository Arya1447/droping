from waypoint_validator import WaypointValidator, waypoint_spatially_passed


def test_sequence_only_not_enough_no_release():
    # spec section 105 case 1: seq=5, required=6 -> NO RELEASE
    wv = WaypointValidator(required_waypoint_seq=6)
    passed = wv.update(
        mission_seq=5, current_position=(100.0, 0.0),
        waypoint_prev_position=(0.0, 0.0), waypoint_current_position=(200.0, 0.0),
    )
    assert passed is False


def test_seq_reached_but_not_spatially_passed():
    # spec section 105 case 2
    wv = WaypointValidator(required_waypoint_seq=6)
    passed = wv.update(
        mission_seq=6, current_position=(50.0, 0.0),  # short of the 200m waypoint
        waypoint_prev_position=(0.0, 0.0), waypoint_current_position=(200.0, 0.0),
    )
    assert passed is False


def test_seq_reached_and_spatially_passed():
    # spec section 105 case 3
    wv = WaypointValidator(required_waypoint_seq=6)
    wv.update(
        mission_seq=6, current_position=(50.0, 0.0),
        waypoint_prev_position=(0.0, 0.0), waypoint_current_position=(200.0, 0.0),
    )
    passed = wv.update(
        mission_seq=6, current_position=(250.0, 0.0),
        waypoint_prev_position=(0.0, 0.0), waypoint_current_position=(200.0, 0.0),
    )
    assert passed is True


def test_latch_remains_true_after_regression_noise():
    wv = WaypointValidator(required_waypoint_seq=6)
    wv.update(6, (50.0, 0.0), (0.0, 0.0), (200.0, 0.0))
    wv.update(6, (250.0, 0.0), (0.0, 0.0), (200.0, 0.0))
    assert wv.passed is True
    # noisy telemetry reporting aircraft "behind" again must not unlatch
    passed = wv.update(6, (10.0, 0.0), (0.0, 0.0), (200.0, 0.0))
    assert passed is True
    assert wv.passed is True


def test_waypoint_spatially_passed_pure_function():
    assert waypoint_spatially_passed((0.0, 0.0), (250.0, 0.0), (0.0, 0.0), (200.0, 0.0)) is True
    assert waypoint_spatially_passed((0.0, 0.0), (50.0, 0.0), (0.0, 0.0), (200.0, 0.0)) is False
