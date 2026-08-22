import math

from so101_dashboard.utils.conversions import (
    all_finite, clamp, deg_to_rad, has_all_joints, is_finite,
    joint_state_to_dict, missing_joints, rad_to_deg, trapezoidal_duration,
    within_limits,
)


def test_rad_deg_round_trip():
    for value in (0.0, 1.0, -1.0, math.pi, -math.pi / 2):
        assert abs(deg_to_rad(rad_to_deg(value)) - value) < 1e-9


def test_is_finite():
    assert is_finite(1.0)
    assert is_finite(0)
    assert not is_finite(float('nan'))
    assert not is_finite(float('inf'))
    assert not is_finite(float('-inf'))


def test_all_finite():
    assert all_finite([1.0, 2.0, 3.0])
    assert not all_finite([1.0, float('nan'), 3.0])
    assert all_finite([])


def test_joint_state_to_dict_filters_and_maps_by_name():
    result = joint_state_to_dict(
        joint_names=['a', 'b'],
        msg_names=['b', 'a', 'c'],
        msg_values=[2.0, 1.0, 3.0],
    )
    assert result == {'a': 1.0, 'b': 2.0}


def test_joint_state_to_dict_missing_joint_is_omitted():
    result = joint_state_to_dict(joint_names=['a', 'b'], msg_names=['a'], msg_values=[1.0])
    assert result == {'a': 1.0}


def test_has_all_joints():
    assert has_all_joints({'a': 1.0, 'b': 2.0}, ['a', 'b'])
    assert not has_all_joints({'a': 1.0}, ['a', 'b'])


def test_missing_joints():
    assert missing_joints({'a': 1.0}, ['a', 'b', 'c']) == ['b', 'c']
    assert missing_joints({'a': 1.0, 'b': 2.0, 'c': 3.0}, ['a', 'b']) == []


def test_within_limits():
    assert within_limits(0.0, -1.0, 1.0)
    assert within_limits(-1.0, -1.0, 1.0)
    assert within_limits(1.0, -1.0, 1.0)
    assert not within_limits(1.1, -1.0, 1.0)
    assert not within_limits(-1.1, -1.0, 1.0)
    assert within_limits(5.0, None, None)
    assert within_limits(5.0, None, 10.0)
    assert not within_limits(11.0, None, 10.0)


def test_clamp():
    assert clamp(5, 0, 10) == 5
    assert clamp(-5, 0, 10) == 0
    assert clamp(15, 0, 10) == 10


def test_trapezoidal_duration_zero_distance():
    assert trapezoidal_duration(0.0, max_velocity=3.0, max_acceleration=6.0) == 0.0


def test_trapezoidal_duration_zero_limits_is_safe():
    # Shouldn't divide by zero / blow up if limits are unset (0).
    assert trapezoidal_duration(1.0, max_velocity=0.0, max_acceleration=6.0) == 0.0
    assert trapezoidal_duration(1.0, max_velocity=3.0, max_acceleration=0.0) == 0.0


def test_trapezoidal_duration_reaches_cruise_speed():
    # Large distance: v_max=3, a_max=6 -> ramp_distance = 3^2/6 = 1.5.
    # distance=10 >= 1.5, so trapezoidal: t = d/v_max + v_max/a_max
    v_max, a_max, d = 3.0, 6.0, 10.0
    expected = d / v_max + v_max / a_max
    assert abs(trapezoidal_duration(d, v_max, a_max) - expected) < 1e-9
    assert expected > d / v_max  # must be slower than an instant-accel move


def test_trapezoidal_duration_never_reaches_cruise_speed():
    # Small distance: v_max=3, a_max=6 -> ramp_distance = 1.5. distance=0.5 < 1.5,
    # so triangular profile: t = 2*sqrt(d/a_max).
    v_max, a_max, d = 3.0, 6.0, 0.5
    expected = 2.0 * (d / a_max) ** 0.5
    assert abs(trapezoidal_duration(d, v_max, a_max) - expected) < 1e-9


def test_trapezoidal_duration_symmetric_in_direction():
    assert trapezoidal_duration(-2.0, 3.0, 6.0) == trapezoidal_duration(2.0, 3.0, 6.0)


def test_trapezoidal_duration_lower_acceleration_takes_longer():
    d, v_max = 5.0, 3.0
    t_fast_accel = trapezoidal_duration(d, v_max, max_acceleration=10.0)
    t_slow_accel = trapezoidal_duration(d, v_max, max_acceleration=2.0)
    assert t_slow_accel > t_fast_accel
