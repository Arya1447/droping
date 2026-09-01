from mavlink_interface import (
    EKF_ATTITUDE, EKF_CONST_POS_MODE, EKF_POS_HORIZ_ABS, EKF_VELOCITY_HORIZ,
    ekf_valid_from_flags, gps_valid_from_fix_type,
)


def test_gps_valid_requires_3d_fix_or_better():
    assert gps_valid_from_fix_type(None) is False
    assert gps_valid_from_fix_type(0) is False  # NO_GPS
    assert gps_valid_from_fix_type(1) is False  # NO_FIX
    assert gps_valid_from_fix_type(2) is False  # 2D_FIX
    assert gps_valid_from_fix_type(3) is True   # 3D_FIX
    assert gps_valid_from_fix_type(6) is True   # RTK_FIXED


def test_ekf_valid_requires_attitude_velocity_and_position():
    good_flags = EKF_ATTITUDE | EKF_VELOCITY_HORIZ | EKF_POS_HORIZ_ABS
    assert ekf_valid_from_flags(good_flags) is True
    assert ekf_valid_from_flags(None) is False
    assert ekf_valid_from_flags(0) is False
    assert ekf_valid_from_flags(EKF_ATTITUDE) is False  # missing velocity/position bits


def test_ekf_invalid_in_const_pos_mode_even_with_other_flags_set():
    flags = EKF_ATTITUDE | EKF_VELOCITY_HORIZ | EKF_POS_HORIZ_ABS | EKF_CONST_POS_MODE
    assert ekf_valid_from_flags(flags) is False
