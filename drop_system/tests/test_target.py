from target_validator import TargetBox, is_inside_target_box, target_ahead, target_valid


def test_target_ahead():
    assert target_ahead(50.0) is True
    assert target_ahead(0.0) is False
    assert target_ahead(-5.0) is False


def test_target_box_inside():
    box = TargetBox(north_m=5, south_m=5, east_m=5, west_m=5)
    assert is_inside_target_box(0.0, 0.0, box) is True
    assert is_inside_target_box(4.9, -4.9, box) is True


def test_target_box_outside():
    box = TargetBox(north_m=5, south_m=5, east_m=5, west_m=5)
    assert is_inside_target_box(6.0, 0.0, box) is False
    assert is_inside_target_box(0.0, -6.0, box) is False


def test_target_valid_and_invalid():
    assert target_valid(-6.9, 107.6) is True
    assert target_valid(None, 107.6) is False
    assert target_valid(200.0, 107.6) is False
    assert target_valid(float("nan"), 107.6) is False
