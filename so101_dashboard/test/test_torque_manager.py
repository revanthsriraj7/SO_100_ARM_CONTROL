"""Unit tests for the safety-critical TorqueManager state machine, per the
scenarios called out in the project spec: ENABLED -> FREE MOVE,
FREE MOVE -> CAPTURE -> HOLD, service failure, double click, stale joint
state, and invalid (out-of-limits) joint state.

Uses FakeRobotInterface (conftest.py) instead of a real ROS service/action
client, and a real JointLimits instance -- no rclpy, no physical robot.
"""
from PySide6.QtTest import QTest

from so101_dashboard.robot.joint_limits import JointLimit, JointLimits
from so101_dashboard.robot.torque_manager import TorqueManager, TorqueState
from so101_dashboard.utils.signals import JointStateSnapshot

ARM_JOINTS = ['j1', 'j2']
GRIPPER = 'gripper'

MOTION_CONFIG = {'hold_trajectory_duration': 0.5, 'gripper_hold_max_effort': 50.0}
SAFETY_CONFIG = {'joint_state_timeout': 0.1, 'action_goal_timeout': 1.0}


def make_limits():
    return JointLimits({
        'j1': JointLimit(min_position=-3.14, max_position=3.14),
        'j2': JointLimit(min_position=-3.14, max_position=3.14),
        'gripper': JointLimit(min_position=-0.2, max_position=1.6),
    })


def make_manager(fake_ri, limits=None):
    return TorqueManager(
        fake_ri, limits or make_limits(), ARM_JOINTS, GRIPPER, MOTION_CONFIG, SAFETY_CONFIG,
    )


def send_joint_state(fake_ri, j1=0.1, j2=0.2, gripper=0.3):
    fake_ri.joint_state_received.emit(JointStateSnapshot(
        stamp_sec=1.0,
        positions={'j1': j1, 'j2': j2, 'gripper': gripper},
        velocities={}, efforts={},
    ))


def test_starts_unknown(fake_ri):
    tm = make_manager(fake_ri)
    assert tm.state == TorqueState.UNKNOWN


def test_enabled_to_free_move(fake_ri):
    tm = make_manager(fake_ri)
    # UNKNOWN is an accepted starting point for FREE MOVE too (spec: must
    # be reachable without ever auto-probing torque state).
    tm.enter_free_move()
    assert tm.state == TorqueState.TOGGLING
    assert fake_ri.cancel_all_calls == 1
    assert len(fake_ri.toggle_torque_calls) == 1

    fake_ri.succeed_last_toggle()
    assert tm.state == TorqueState.FREE_MOVE


def test_free_move_capture_hold_full_cycle(fake_ri):
    tm = make_manager(fake_ri)
    tm.enter_free_move()
    fake_ri.succeed_last_toggle()
    assert tm.state == TorqueState.FREE_MOVE

    tm.hold_current_position()
    assert tm.state == TorqueState.CAPTURING_POSITION

    send_joint_state(fake_ri)  # -- CAPTURE --
    assert tm.state == TorqueState.TOGGLING  # validated -> PREPARING_HOLD -> TOGGLING
    assert len(fake_ri.toggle_torque_calls) == 2

    fake_ri.succeed_last_toggle()  # -- HOLD: torque re-enabled --
    assert len(fake_ri.arm_goal_calls) == 1
    names, positions, duration, _ = fake_ri.arm_goal_calls[0]
    assert names == ARM_JOINTS
    assert positions == [0.1, 0.2]
    assert tm.state == TorqueState.TOGGLING  # waiting for hold goal to be accepted

    fake_ri.succeed_last_arm_goal()
    assert tm.state == TorqueState.HOLDING  # -- ENABLED / HOLDING --


def test_service_failure_goes_to_error(fake_ri):
    tm = make_manager(fake_ri)
    tm.enter_free_move()
    errors = []
    tm.error_occurred.connect(errors.append)

    fake_ri.fail_last_toggle('servo bus timeout')
    assert tm.state == TorqueState.ERROR
    assert errors and 'servo bus timeout' in errors[0]

    # never silently assumed torque changed
    tm.acknowledge_error()
    assert tm.state == TorqueState.UNKNOWN


def test_double_click_does_not_double_toggle(fake_ri):
    tm = make_manager(fake_ri)
    tm.enter_free_move()
    assert len(fake_ri.toggle_torque_calls) == 1

    # second press while TOGGLING must be ignored, not queued
    tm.enter_free_move()
    assert len(fake_ri.toggle_torque_calls) == 1
    assert tm.state == TorqueState.TOGGLING

    fake_ri.succeed_last_toggle()
    assert tm.state == TorqueState.FREE_MOVE

    # hold_current_position while not FREE_MOVE/DISABLED must be ignored
    tm2 = make_manager(fake_ri)
    tm2.hold_current_position()
    assert tm2.state == TorqueState.UNKNOWN  # unchanged, request ignored


def test_stale_joint_state_blocks_hold(fake_ri, qapp):
    tm = make_manager(fake_ri)
    tm.enter_free_move()
    fake_ri.succeed_last_toggle()

    errors = []
    tm.error_occurred.connect(errors.append)

    tm.hold_current_position()
    assert tm.state == TorqueState.CAPTURING_POSITION
    # deliberately never send a joint state -- wait past the (short, test-
    # configured) timeout
    QTest.qWait(int(SAFETY_CONFIG['joint_state_timeout'] * 1000) + 1200)

    assert tm.state == TorqueState.ERROR
    assert errors and 'stale' in errors[0].lower()
    # torque was never re-toggled -- still only the original FREE MOVE call
    assert len(fake_ri.toggle_torque_calls) == 1


def test_invalid_joint_state_blocks_hold(fake_ri):
    tm = make_manager(fake_ri)
    tm.enter_free_move()
    fake_ri.succeed_last_toggle()

    errors = []
    tm.error_occurred.connect(errors.append)

    tm.hold_current_position()
    # j1 way outside its [-3.14, 3.14] limit
    send_joint_state(fake_ri, j1=99.0)

    assert tm.state == TorqueState.ERROR
    assert errors and 'j1' in errors[0]
    # torque was never re-toggled to re-enable on bad data
    assert len(fake_ri.toggle_torque_calls) == 1
    assert len(fake_ri.arm_goal_calls) == 0


def test_hold_goal_rejected_after_torque_reenabled_is_reported(fake_ri):
    tm = make_manager(fake_ri)
    tm.enter_free_move()
    fake_ri.succeed_last_toggle()
    tm.hold_current_position()
    send_joint_state(fake_ri)
    fake_ri.succeed_last_toggle()  # torque re-enabled

    errors = []
    tm.error_occurred.connect(errors.append)
    fake_ri.reject_last_arm_goal('controller not accepting goals')

    assert tm.state == TorqueState.ERROR
    assert errors and 'rejected' in errors[0].lower()


# -- gripper lock (optional, independent of the main torque state machine) --

def test_gripper_lock_defaults_to_enabled(fake_ri):
    tm = make_manager(fake_ri)
    assert tm.gripper_lock_enabled is True


def test_set_gripper_lock_calls_service(fake_ri):
    tm = make_manager(fake_ri)
    tm.set_gripper_lock(False)
    assert len(fake_ri.gripper_lock_calls) == 1
    locked, request_id = fake_ri.gripper_lock_calls[0]
    assert locked is False
    assert request_id


def test_set_gripper_lock_updates_confirmed_state_on_success(fake_ri):
    tm = make_manager(fake_ri)
    changed = []
    tm.gripper_lock_changed.connect(changed.append)

    tm.set_gripper_lock(False)
    assert tm.gripper_lock_enabled is True  # not yet confirmed

    fake_ri.succeed_last_gripper_lock()
    assert tm.gripper_lock_enabled is False
    assert changed == [False]


def test_set_gripper_lock_failure_leaves_state_unchanged_and_reports_error(fake_ri):
    tm = make_manager(fake_ri)
    errors = []
    tm.error_occurred.connect(errors.append)
    changed = []
    tm.gripper_lock_changed.connect(changed.append)

    tm.set_gripper_lock(False)
    fake_ri.fail_last_gripper_lock('service unavailable')

    assert tm.gripper_lock_enabled is True  # unchanged, not confirmed
    assert not changed
    assert errors and 'service unavailable' in errors[0]


def test_set_gripper_lock_independent_of_main_torque_state(fake_ri):
    # Can be toggled at any time, including mid-FREE-MOVE, without
    # disturbing the main torque state machine at all.
    tm = make_manager(fake_ri)
    tm.enter_free_move()
    fake_ri.succeed_last_toggle()
    assert tm.state == TorqueState.FREE_MOVE

    tm.set_gripper_lock(False)
    fake_ri.succeed_last_gripper_lock()

    assert tm.gripper_lock_enabled is False
    assert tm.state == TorqueState.FREE_MOVE  # untouched


def test_stray_gripper_lock_result_is_ignored(fake_ri):
    tm = make_manager(fake_ri)
    tm.set_gripper_lock(False)
    fake_ri.succeed_last_gripper_lock()
    assert tm.gripper_lock_enabled is False

    # A late result for some earlier/unrelated request_id must not apply.
    from so101_dashboard.utils.signals import ServiceCallOutcome
    tm._on_gripper_lock_result(ServiceCallOutcome(success=True, message='ok', request_id='stale-id'))
    assert tm.gripper_lock_enabled is False  # unchanged
