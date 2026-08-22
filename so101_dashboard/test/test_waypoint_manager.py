"""Unit tests for WaypointManager: record/edit operations, save/load
(waypoint serialization) round trip, and that MOVE TO refuses to send a
goal for an out-of-limits waypoint.
"""
import os

from so101_dashboard.robot.joint_limits import JointLimit, JointLimits
from so101_dashboard.robot.waypoint_manager import WaypointManager

ARM_JOINTS = ['j1', 'j2']
GRIPPER = 'gripper'


class FakeRobotState:
    def __init__(self, positions):
        self.positions = positions


def make_limits():
    return JointLimits({
        'j1': JointLimit(min_position=-3.14, max_position=3.14),
        'j2': JointLimit(min_position=-3.14, max_position=3.14),
        'gripper': JointLimit(min_position=-0.2, max_position=1.6),
    })


def make_limits_with_kinematics():
    # Same position limits, but with velocity/acceleration set so
    # min_move_duration() actually has something to compute from.
    return JointLimits({
        'j1': JointLimit(min_position=-3.14, max_position=3.14, max_velocity=3.0, max_acceleration=6.0),
        'j2': JointLimit(min_position=-3.14, max_position=3.14, max_velocity=3.0, max_acceleration=6.0),
        'gripper': JointLimit(min_position=-0.2, max_position=1.6, max_velocity=3.0, max_acceleration=6.0),
    })


def make_manager(fake_ri, positions=None, limits=None):
    rs = FakeRobotState(positions or {'j1': 0.1, 'j2': 0.2, 'gripper': 0.3})
    return WaypointManager(fake_ri, rs, limits or make_limits(), ARM_JOINTS, GRIPPER), rs


def test_record_captures_current_positions(fake_ri):
    wm, _ = make_manager(fake_ri)
    changed = []
    wm.waypoints_changed.connect(lambda: changed.append(True))

    assert wm.record('home') is True
    assert len(wm.waypoints) == 1
    wp = wm.waypoints[0]
    assert wp.name == 'home'
    assert wp.joint_names == ['j1', 'j2', 'gripper']
    assert wp.joint_positions == [0.1, 0.2, 0.3]
    assert changed


def test_record_fails_cleanly_if_joint_missing(fake_ri):
    wm, _ = make_manager(fake_ri, positions={'j1': 0.1})  # missing j2, gripper
    errors = []
    wm.error_occurred.connect(errors.append)

    assert wm.record('home') is False
    assert len(wm.waypoints) == 0
    assert errors


def test_rename_delete_duplicate(fake_ri):
    wm, _ = make_manager(fake_ri)
    wm.record('a')
    wm.record('b')

    wm.rename(0, 'a_renamed')
    assert wm.waypoints[0].name == 'a_renamed'

    wm.duplicate(0)
    assert len(wm.waypoints) == 3
    assert wm.waypoints[1].name == 'a_renamed_copy'

    wm.delete(1)
    assert len(wm.waypoints) == 2
    assert [w.name for w in wm.waypoints] == ['a_renamed', 'b']


def test_reorder(fake_ri):
    wm, _ = make_manager(fake_ri)
    wm.record('a')
    wm.record('b')
    wm.record('c')

    wm.move_item_down(0)
    assert [w.name for w in wm.waypoints] == ['b', 'a', 'c']

    wm.move_item_up(2)
    assert [w.name for w in wm.waypoints] == ['b', 'c', 'a']

    # boundaries are no-ops, not errors
    wm.move_item_up(0)
    assert [w.name for w in wm.waypoints] == ['b', 'c', 'a']
    wm.move_item_down(2)
    assert [w.name for w in wm.waypoints] == ['b', 'c', 'a']


def test_save_load_round_trip(fake_ri, tmp_path):
    wm, _ = make_manager(fake_ri)
    wm.record('a')
    wm.record('b')
    path = os.path.join(tmp_path, 'waypoints.yaml')

    wm.save(path)
    wm.clear()
    assert len(wm.waypoints) == 0

    wm.load(path)
    assert [w.name for w in wm.waypoints] == ['a', 'b']
    assert wm.waypoints[0].joint_positions == [0.1, 0.2, 0.3]


def test_move_to_rejects_out_of_limits_waypoint(fake_ri):
    wm, rs = make_manager(fake_ri)
    wm.record('bad')
    wm.waypoints[0].joint_positions[0] = 99.0  # blow past j1's limit

    errors = []
    wm.error_occurred.connect(errors.append)
    wm.move_to(0, duration_sec=2.0)

    assert errors
    assert len(fake_ri.arm_goal_calls) == 0  # never sent


def test_move_to_sends_goal_for_valid_waypoint(fake_ri):
    wm, _ = make_manager(fake_ri)
    wm.record('good')

    wm.move_to(0, duration_sec=2.0, velocity_scaling=0.5)

    assert len(fake_ri.arm_goal_calls) == 1
    names, positions, duration, _ = fake_ri.arm_goal_calls[0]
    assert names == ARM_JOINTS
    assert positions == [0.1, 0.2]
    assert duration == 4.0  # 2.0s / 0.5 scaling; no kinematic limits configured here
    assert len(fake_ri.gripper_goal_calls) == 1


def test_move_to_ignores_acceleration_scaling_without_kinematic_limits(fake_ri):
    # make_limits() (no max_velocity/max_acceleration) -> physical_min_duration
    # is always 0, so acceleration_scaling can't affect anything -- confirms
    # it doesn't silently do something when there's nothing to compute from.
    wm, _ = make_manager(fake_ri)
    wm.record('wp')
    wm.move_to(0, duration_sec=2.0, velocity_scaling=1.0, acceleration_scaling=0.05)
    _, _, duration, _ = fake_ri.arm_goal_calls[0]
    assert duration == 2.0


def test_move_to_acceleration_scaling_extends_short_moves(fake_ri):
    # Tiny displacement -> velocity-scaled duration is trivially small
    # (floored at 0.1s), but the acceleration-limited minimum time should
    # dominate and a lower acceleration_scaling should make it take longer.
    wm, _ = make_manager(
        fake_ri, positions={'j1': 0.10, 'j2': 0.20, 'gripper': 0.30},
        limits=make_limits_with_kinematics(),
    )
    wm.record('close_by')
    wm.waypoints[0].joint_positions[0] = 0.15  # j1 moves 0.05 rad -- tiny

    wm.move_to(0, duration_sec=0.1, velocity_scaling=1.0, acceleration_scaling=1.0)
    _, _, duration_full_accel, _ = fake_ri.arm_goal_calls[0]

    wm.move_to(0, duration_sec=0.1, velocity_scaling=1.0, acceleration_scaling=0.1)
    _, _, duration_low_accel, _ = fake_ri.arm_goal_calls[1]

    assert duration_low_accel > duration_full_accel
    assert duration_full_accel > 0.1  # exceeds the floor -- acceleration limit is binding


# -- add_existing: importing a waypoint from elsewhere (e.g. a loaded Program) --

def test_add_existing_appends_a_copy(fake_ri):
    from so101_dashboard.storage.waypoint_storage import Waypoint
    wm, _ = make_manager(fake_ri)
    source = Waypoint.create('Imported', ['j1', 'j2', 'gripper'], [1.0, 2.0, 3.0])

    assert wm.add_existing(source) is True
    assert len(wm.waypoints) == 1
    assert wm.waypoints[0].name == 'Imported'
    assert wm.waypoints[0].joint_positions == [1.0, 2.0, 3.0]
    # confirms it's a copy, not a shared reference
    source.name = 'mutated'
    assert wm.waypoints[0].name == 'Imported'


def test_add_existing_skips_duplicate_name(fake_ri):
    from so101_dashboard.storage.waypoint_storage import Waypoint
    wm, _ = make_manager(fake_ri)
    wm.add_existing(Waypoint.create('Home', ['j1'], [0.0]))

    added = wm.add_existing(Waypoint.create('Home', ['j1'], [99.0]))

    assert added is False
    assert len(wm.waypoints) == 1
    assert wm.waypoints[0].joint_positions == [0.0]  # first one wins, not overwritten


# -- update_or_add: pushing a re-recorded Program waypoint's correction into the library --

def test_update_or_add_overwrites_existing_entry_by_name(fake_ri):
    from so101_dashboard.storage.waypoint_storage import Waypoint
    wm, _ = make_manager(fake_ri)
    wm.add_existing(Waypoint.create('Pick', ['j1'], [0.1]))

    wm.update_or_add(Waypoint.create('Pick', ['j1'], [0.9]))

    assert len(wm.waypoints) == 1  # updated in place, not duplicated
    assert wm.waypoints[0].joint_positions == [0.9]


def test_update_or_add_appends_when_no_matching_name(fake_ri):
    from so101_dashboard.storage.waypoint_storage import Waypoint
    wm, _ = make_manager(fake_ri)

    wm.update_or_add(Waypoint.create('New', ['j1'], [0.5]))

    assert len(wm.waypoints) == 1
    assert wm.waypoints[0].name == 'New'
    assert wm.waypoints[0].joint_positions == [0.5]


def test_update_or_add_emits_waypoints_changed(fake_ri):
    from so101_dashboard.storage.waypoint_storage import Waypoint
    wm, _ = make_manager(fake_ri)
    changed = []
    wm.waypoints_changed.connect(lambda: changed.append(True))

    wm.update_or_add(Waypoint.create('X', ['j1'], [0.0]))
    assert changed

    changed.clear()
    wm.update_or_add(Waypoint.create('X', ['j1'], [1.0]))
    assert changed
