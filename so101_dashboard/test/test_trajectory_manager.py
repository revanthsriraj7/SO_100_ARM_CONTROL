"""Unit tests for TrajectoryManager (playback validation/state machine)
and trajectory serialization round-trip.
"""
import os

from so101_dashboard.robot.joint_limits import JointLimit, JointLimits
from so101_dashboard.robot.trajectory_manager import PlaybackState, TrajectoryManager
from so101_dashboard.storage.trajectory_storage import (
    TrajectoryData, TrajectoryPointData, load_trajectory, save_trajectory,
)

JOINTS = ['j1', 'j2']
GRIPPER = 'gripper'


def make_limits():
    return JointLimits({
        'j1': JointLimit(min_position=-3.14, max_position=3.14),
        'j2': JointLimit(min_position=-3.14, max_position=3.14),
        'gripper': JointLimit(min_position=-0.2, max_position=1.6),
    })


def make_manager(fake_ri):
    return TrajectoryManager(fake_ri, make_limits(), JOINTS, GRIPPER)


def make_trajectory(positions_list, joint_names=None):
    return TrajectoryData(
        name='t',
        joint_names=joint_names or JOINTS,
        points=[
            TrajectoryPointData(time_from_start=float(i), positions=list(p))
            for i, p in enumerate(positions_list)
        ],
        sample_rate_hz=10.0,
    )


# -- serialization --

def test_trajectory_save_load_round_trip(tmp_path):
    traj = make_trajectory([[0.0, 0.0], [0.5, 0.5], [1.0, 1.0]])
    path = os.path.join(tmp_path, 'traj.yaml')

    save_trajectory(path, traj)
    loaded = load_trajectory(path)

    assert loaded.name == traj.name
    assert loaded.joint_names == traj.joint_names
    assert loaded.sample_rate_hz == traj.sample_rate_hz
    assert len(loaded.points) == 3
    assert loaded.points[1].positions == [0.5, 0.5]
    assert loaded.duration == 2.0


def test_trajectory_duration_empty():
    assert TrajectoryData(name='t', joint_names=JOINTS, points=[]).duration == 0.0


# -- validation --

def test_validate_no_trajectory_loaded(fake_ri):
    tm = make_manager(fake_ri)
    errors = tm.validate()
    assert errors and 'no trajectory' in errors[0].lower()


def test_validate_empty_points(fake_ri):
    tm = make_manager(fake_ri)
    tm.load(TrajectoryData(name='t', joint_names=JOINTS, points=[]))
    errors = tm.validate()
    assert errors and 'no points' in errors[0].lower()


def test_validate_position_count_mismatch(fake_ri):
    tm = make_manager(fake_ri)
    tm.load(TrajectoryData(
        name='t', joint_names=JOINTS,
        points=[TrajectoryPointData(time_from_start=0.0, positions=[0.0])],  # only 1, need 2
    ))
    errors = tm.validate()
    assert errors and 'position' in errors[0].lower()


def test_validate_out_of_limits_point(fake_ri):
    tm = make_manager(fake_ri)
    tm.load(make_trajectory([[0.0, 0.0], [99.0, 0.0]]))
    errors = tm.validate()
    assert errors and 'j1' in errors[0]


def test_play_refuses_invalid_trajectory(fake_ri):
    tm = make_manager(fake_ri)
    tm.load(make_trajectory([[99.0, 0.0]]))
    errors = []
    tm.error_occurred.connect(errors.append)

    tm.play()

    assert errors
    assert len(fake_ri.arm_goal_calls) == 0
    assert tm.state == PlaybackState.IDLE


def test_play_valid_trajectory_sends_goal_and_reaches_idle_on_success(fake_ri):
    tm = make_manager(fake_ri)
    tm.load(make_trajectory([[0.0, 0.0], [0.5, 0.5]]))

    tm.play()
    assert tm.state == PlaybackState.PLAYING
    assert len(fake_ri.arm_goal_calls) == 1

    fake_ri.succeed_last_arm_goal()
    assert tm.state == PlaybackState.IDLE


def test_play_rejected_goal_goes_to_idle_with_error(fake_ri):
    tm = make_manager(fake_ri)
    tm.load(make_trajectory([[0.0, 0.0], [0.5, 0.5]]))
    errors = []
    tm.error_occurred.connect(errors.append)

    tm.play()
    fake_ri.reject_last_arm_goal('busy')

    assert tm.state == PlaybackState.IDLE
    assert errors and 'rejected' in errors[0].lower()


def test_stop_cancels_and_resets(fake_ri):
    tm = make_manager(fake_ri)
    tm.load(make_trajectory([[0.0, 0.0], [0.5, 0.5]]))
    tm.play()

    tm.stop()

    assert fake_ri.cancel_all_calls == 1
    assert tm.state == PlaybackState.IDLE


# -- arm/gripper split: the actual bug this whole class of test guards --
# against -- a recorded trajectory's joint_names is arm joints AND the
# gripper together (see TrajectoryRecorder), but arm_controller's
# FollowJointTrajectory doesn't manage the gripper joint at all. Sending
# the whole recorded joint list as one goal got the goal rejected outright
# by arm_controller, every single time playback was attempted.

def test_play_without_gripper_in_recording_sends_only_arm_joints(fake_ri):
    tm = make_manager(fake_ri)
    tm.load(make_trajectory([[0.0, 0.0], [0.5, 0.5]]))  # JOINTS only, no gripper

    tm.play()

    names, points, _, _ = fake_ri.arm_goal_calls[0]
    assert names == JOINTS
    assert len(fake_ri.gripper_goal_calls) == 0  # nothing to move, wasn't recorded


def test_play_with_gripper_in_recording_splits_arm_and_gripper(fake_ri):
    tm = make_manager(fake_ri)
    traj = make_trajectory(
        [[0.0, 0.0, 0.2], [0.5, 0.5, 0.9]],
        joint_names=JOINTS + [GRIPPER],
    )
    tm.load(traj)

    tm.play()

    # arm_controller only ever sees the arm joints -- this is the fix
    names, points, _, _ = fake_ri.arm_goal_calls[0]
    assert names == JOINTS
    assert len(points) == 2
    assert points[0][1] == [0.0, 0.0]  # (time, positions, velocities) -- arm positions only
    assert points[1][1] == [0.5, 0.5]

    # gripper gets a one-shot move to its position at the start of this segment
    assert len(fake_ri.gripper_goal_calls) == 1
    position, _max_effort, request_id = fake_ri.gripper_goal_calls[0]
    assert position == 0.2
    assert request_id.endswith('-gripper')


def test_play_resume_sends_gripper_position_for_the_resumed_segment(fake_ri):
    tm = make_manager(fake_ri)
    traj = make_trajectory(
        [[0.0, 0.0, 0.2], [0.3, 0.3, 0.5], [0.6, 0.6, 0.9]],
        joint_names=JOINTS + [GRIPPER],
    )
    tm.load(traj)

    tm._send_from_index(1)  # simulate resuming from the middle point

    position, _, _ = fake_ri.gripper_goal_calls[0]
    assert position == 0.5  # the gripper value AT the resume point, not point 0


def test_play_missing_arm_joint_in_recording_fails_cleanly(fake_ri):
    tm = make_manager(fake_ri)
    # Recording only has j1 -- j2 (part of JOINTS) is missing entirely.
    traj = TrajectoryData(
        name='t', joint_names=['j1'],
        points=[TrajectoryPointData(time_from_start=0.0, positions=[0.0])],
    )
    tm.load(traj)
    errors = []
    tm.error_occurred.connect(errors.append)

    tm.play()

    assert errors
    assert len(fake_ri.arm_goal_calls) == 0
    assert tm.state == PlaybackState.IDLE
