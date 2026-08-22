"""Unit tests for GripperManager: open/close/position use the same
config-driven positions as the rest of the app, validate against
JointLimits, and report completion via goal_finished."""
from so101_dashboard.robot.gripper_manager import GripperManager
from so101_dashboard.robot.joint_limits import JointLimit, JointLimits
from so101_dashboard.utils.signals import ActionOutcome

GRIPPER = 'gripper'
MOTION_CONFIG = {
    'gripper_open_position': 1.5,
    'gripper_closed_position': 0.0,
    'gripper_hold_max_effort': 50.0,
}


def make_limits():
    return JointLimits({'gripper': JointLimit(min_position=-0.2, max_position=1.6)})


def test_open_sends_configured_open_position(fake_ri):
    gm = GripperManager(fake_ri, make_limits(), GRIPPER, MOTION_CONFIG)
    gm.open()
    assert fake_ri.gripper_goal_calls[-1][:2] == (1.5, 50.0)


def test_close_sends_configured_closed_position(fake_ri):
    gm = GripperManager(fake_ri, make_limits(), GRIPPER, MOTION_CONFIG)
    gm.close()
    assert fake_ri.gripper_goal_calls[-1][:2] == (0.0, 50.0)


def test_move_to_rejects_out_of_limits(fake_ri):
    gm = GripperManager(fake_ri, make_limits(), GRIPPER, MOTION_CONFIG)
    errors = []
    gm.error_occurred.connect(errors.append)
    try:
        gm.move_to(99.0)
        assert False, 'expected ValueError'
    except ValueError:
        pass
    assert errors
    assert fake_ri.gripper_goal_calls == []


def test_goal_finished_emits_matching_result_only(fake_ri):
    gm = GripperManager(fake_ri, make_limits(), GRIPPER, MOTION_CONFIG)
    results = []
    gm.goal_finished.connect(results.append)
    gm.open()

    # A stray/unrelated result (some other request's id) must be ignored.
    fake_ri.trajectory_result.emit(
        ActionOutcome(accepted=True, succeeded=True, message='unrelated', request_id='not-the-one')
    )
    assert results == []

    # The result matching the request_id GripperManager actually sent does fire it.
    sent_request_id = fake_ri.gripper_goal_calls[-1][2]
    fake_ri.trajectory_result.emit(
        ActionOutcome(accepted=True, succeeded=True, message='ok', request_id=sent_request_id)
    )
    assert len(results) == 1
    assert results[0].message == 'ok'
