"""Unit tests for robot/kinematics.py.

The forward() reference values below aren't just "whatever the code
happens to compute" -- they were independently cross-checked against
ROS's own robot_state_publisher + tf2 (which computes TF from the real
URDF via kdl_parser) at each of these three joint configurations, and
matched to 6 decimal places on both position and full orientation. See
kinematics.py's module docstring for how. If forward() ever needs to
change, re-verify against tf2 the same way rather than just updating
these numbers to match new code.
"""
import math

from so101_dashboard.robot import kinematics
from so101_dashboard.robot.joint_limits import JointLimit, JointLimits

ARM_JOINTS = ['Shoulder_Rotation', 'Shoulder_Pitch', 'Elbow', 'Wrist_Pitch', 'Wrist_Roll']


def make_limits():
    return JointLimits({
        'Shoulder_Rotation': JointLimit(min_position=-1.96, max_position=1.96),
        'Shoulder_Pitch': JointLimit(min_position=-1.745, max_position=1.745),
        'Elbow': JointLimit(min_position=-1.5, max_position=1.5),
        'Wrist_Pitch': JointLimit(min_position=-1.658, max_position=1.658),
        'Wrist_Roll': JointLimit(min_position=-2.75, max_position=2.75),
    })


# -- forward kinematics: cross-checked against tf2 (see docstring above) --

def test_forward_at_zero_matches_tf2_reference():
    x, y, z = kinematics.forward([0.0, 0.0, 0.0, 0.0, 0.0])
    assert math.isclose(x, 0.0, abs_tol=1e-5)
    assert math.isclose(y, -0.388801, abs_tol=1e-5)
    assert math.isclose(z, 0.236768, abs_tol=1e-5)


def test_forward_at_mid_pose_matches_tf2_reference():
    x, y, z = kinematics.forward([0.3, -0.2, 0.5, 0.1, 0.4])
    assert math.isclose(x, -0.089939, abs_tol=1e-5)
    assert math.isclose(y, -0.335948, abs_tol=1e-5)
    assert math.isclose(z, 0.141537, abs_tol=1e-5)


def test_forward_at_extreme_pose_matches_tf2_reference():
    x, y, z = kinematics.forward([1.0, 1.0, -1.0, 0.5, -0.5])
    assert math.isclose(x, -0.342544, abs_tol=1e-5)
    assert math.isclose(y, -0.265145, abs_tol=1e-5)
    assert math.isclose(z, 0.089497, abs_tol=1e-5)


def test_forward_rejects_wrong_number_of_joints():
    import pytest
    with pytest.raises(ValueError):
        kinematics.forward([0.0, 0.0, 0.0])


# -- inverse kinematics --

def test_ik_small_jog_converges_and_round_trips():
    seed = [0.3, -0.2, 0.5, 0.1, 0.4]
    x, y, z = kinematics.forward(seed)
    target = (x + 0.01, y, z)  # 1cm jog in X
    result = kinematics.solve_ik(seed, target, make_limits(), ARM_JOINTS)
    assert result.success
    assert result.residual_m < 1e-4
    fx, fy, fz = kinematics.forward(result.positions)
    assert math.isclose(fx, target[0], abs_tol=1e-4)
    assert math.isclose(fy, target[1], abs_tol=1e-4)
    assert math.isclose(fz, target[2], abs_tol=1e-4)


def test_ik_repeated_small_jogs_stay_converged():
    q = [0.2, -0.3, 0.6, 0.2, 0.0]
    limits = make_limits()
    for _ in range(10):
        x, y, z = kinematics.forward(q)
        target = (x + 0.005, y, z)
        result = kinematics.solve_ik(q, target, limits, ARM_JOINTS)
        assert result.success, result.message
        q = result.positions


def test_ik_stays_within_joint_limits():
    limits = make_limits()
    seed = [1.9, 0.0, 0.0, 0.0, 0.0]  # near Shoulder_Rotation's limit
    x, y, z = kinematics.forward(seed)
    # push hard in a direction that would need Shoulder_Rotation to keep
    # increasing past its limit
    target = (x, y - 0.2, z)
    result = kinematics.solve_ik(seed, target, limits, ARM_JOINTS)
    if result.success:
        assert result.positions[0] <= 1.96 + 1e-6
        assert result.positions[0] >= -1.96 - 1e-6


def test_ik_unreachable_target_fails_cleanly():
    seed = [0.0, 0.0, 0.0, 0.0, 0.0]
    # 10 meters away -- nowhere near reachable for this arm
    result = kinematics.solve_ik(seed, (10.0, 10.0, 10.0), make_limits(), ARM_JOINTS)
    assert result.success is False
    assert result.positions is None


def test_ik_result_never_returned_as_success_without_forward_verification():
    # Regression guard: success must always mean forward(positions) is
    # actually within tolerance of target -- not just "solver ran".
    seed = [0.1, 0.1, 0.1, 0.1, 0.1]
    x, y, z = kinematics.forward(seed)
    target = (x + 0.02, y - 0.01, z + 0.015)
    result = kinematics.solve_ik(seed, target, make_limits(), ARM_JOINTS)
    if result.success:
        fx, fy, fz = kinematics.forward(result.positions)
        err = math.dist((fx, fy, fz), target)
        assert err < 1e-3
