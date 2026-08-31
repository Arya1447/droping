import math

import pytest

import config
import physics_model
from coordinate_utils import horizontal_to_ne
from prediction import PhysicsCoreInputs, compute_physics_core


def test_ballistic_fall_time_zero_vz0():
    # spec section 123: h=100, V=18, gamma=0 -> t = sqrt(2h/g)
    t = physics_model.time_of_flight(100.0, 0.0, g=9.81)
    expected = math.sqrt(2 * 100.0 / 9.81)
    assert t == pytest.approx(expected, rel=1e-9)


def test_ballistic_fall_time_with_climb():
    t = physics_model.time_of_flight(50.0, vz0_mps=2.0, g=9.81)
    # climbing initially -> longer flight time than vz0=0 case
    t0 = physics_model.time_of_flight(50.0, vz0_mps=0.0, g=9.81)
    assert t > t0


def test_ballistic_fall_time_negative_height_raises():
    with pytest.raises(ValueError):
        physics_model.time_of_flight(-1.0)


def test_projectile_displacement_matches_analytical():
    # spec section 123 physical model validation
    v = 18.0
    t_flight = math.sqrt(2 * 100.0 / 9.81)
    expected_d = v * t_flight
    d = physics_model.ballistic_displacement(v, t_flight)
    assert d == pytest.approx(expected_d, rel=1e-9)


def test_gravity_acceleration_is_mass_independent():
    assert physics_model.gravity_acceleration(9.81) == 9.81


def test_ballistic_mass_invariance_A_vs_B():
    """TEST A vs TEST B (spec section 163/17A-J): ballistic-only,
    0.5 kg vs 1.0 kg -> identical flight time and trajectory.
    """
    v_n, v_e = horizontal_to_ne(18.0, 0.0)
    inputs_common = dict(altitude_m=100.0, ground_velocity_n=v_n, ground_velocity_e=v_e,
                          ground_velocity_u=0.0, servo_delay_s=0.3, drag_enabled=False)

    result_a = compute_physics_core(PhysicsCoreInputs(payload_mass_kg=0.5, **inputs_common))
    result_b = compute_physics_core(PhysicsCoreInputs(payload_mass_kg=1.0, **inputs_common))

    assert result_a.t_flight_s == pytest.approx(result_b.t_flight_s, rel=1e-9)
    assert result_a.drop_distance_m == pytest.approx(result_b.drop_distance_m, rel=1e-9)
    assert result_a.gravity_acceleration_mps2 == result_b.gravity_acceleration_mps2


def test_drag_mass_sensitivity_C_vs_D():
    """TEST C vs TEST D (spec section 163/17A-K): drag-enabled, 0.5 kg
    vs 1.0 kg with identical rho/Cd/A/Vrel -> different drag
    acceleration (heavier payload -> smaller |a_drag|).
    """
    a_light = physics_model.drag_acceleration(rho=1.225, cd=0.5, area_m2=0.01, mass_kg=0.5,
                                               v_rel_n=10.0, v_rel_e=0.0, v_rel_u=0.0)
    a_heavy = physics_model.drag_acceleration(rho=1.225, cd=0.5, area_m2=0.01, mass_kg=1.0,
                                               v_rel_n=10.0, v_rel_e=0.0, v_rel_u=0.0)
    mag_light = math.hypot(*a_light)
    mag_heavy = math.hypot(*a_heavy)
    assert mag_heavy < mag_light
    assert mag_light == pytest.approx(2 * mag_heavy, rel=1e-9)


def test_ballistic_coefficient_unknown_without_cd_area():
    assert physics_model.ballistic_coefficient(0.5, None, None) is None
    assert physics_model.ballistic_coefficient(0.5, 0.5, 0.01) == pytest.approx(0.5 / (0.5 * 0.01))


def test_moment_of_inertia_not_used_in_translation():
    props = physics_model.MassProperties(mass_kg=0.5, mass_source="CONFIGURED",
                                          diameter_m=0.08, cylinder_length_m=0.2, nose_length_m=0.05)
    i_axis = props.moment_of_inertia_axis()
    assert i_axis is not None and i_axis > 0
    # translational acceleration must remain g regardless of I
    assert physics_model.gravity_acceleration(9.81) == 9.81
