"""Unit tests for ProgramManager: element editing and the sequential
execution state machine (ordering, waiting for each step, STOP, failure
handling).
"""
import os

from so101_dashboard.robot.joint_limits import JointLimit, JointLimits
from so101_dashboard.robot.program_manager import ElementState, ExecutionState, ProgramManager
from so101_dashboard.storage.program_storage import ElementType
from so101_dashboard.storage.waypoint_storage import Waypoint

ARM_JOINTS = ['j1', 'j2']
GRIPPER = 'gripper'
MOTION_CONFIG = {
    'gripper_open_position': 1.5,
    'gripper_closed_position': 0.0,
    'gripper_hold_max_effort': 50.0,
    'program_waypoint_duration': 2.0,
}


class FakeRobotState:
    def __init__(self, positions):
        self.positions = positions


def make_limits():
    return JointLimits({
        'j1': JointLimit(min_position=-3.14, max_position=3.14),
        'j2': JointLimit(min_position=-3.14, max_position=3.14),
        'gripper': JointLimit(min_position=-0.2, max_position=1.6),
    })


def make_manager(fake_ri, positions=None):
    rs = FakeRobotState(positions or {'j1': 0.1, 'j2': 0.2, 'gripper': 0.3})
    pm = ProgramManager(fake_ri, rs, make_limits(), ARM_JOINTS, GRIPPER, MOTION_CONFIG)
    return pm, rs


# -- editing --

def test_add_waypoint_records_current_position(fake_ri):
    pm, _ = make_manager(fake_ri)
    changed = []
    pm.program_changed.connect(lambda: changed.append(True))

    assert pm.add_waypoint() is True
    assert len(pm.program.elements) == 1
    el = pm.program.elements[0]
    assert el.element_type == ElementType.WAYPOINT
    assert el.waypoint.joint_positions == [0.1, 0.2, 0.3]
    assert el.waypoint.name == 'Waypoint 1'
    assert changed


def test_add_waypoint_fails_cleanly_if_joint_missing(fake_ri):
    pm, _ = make_manager(fake_ri, positions={'j1': 0.1})
    errors = []
    pm.error_occurred.connect(errors.append)
    assert pm.add_waypoint() is False
    assert len(pm.program.elements) == 0
    assert errors


def test_insertion_position(fake_ri):
    pm, _ = make_manager(fake_ri)
    pm.add_waypoint()  # index 0
    pm.add_jaw_close()  # index 1, appended
    pm.add_jaw_open(index=1)  # inserted BETWEEN waypoint and jaw_close
    types = [e.element_type for e in pm.program.elements]
    assert types == [ElementType.WAYPOINT, ElementType.JAW_OPEN, ElementType.JAW_CLOSE]


def test_add_existing_waypoint_from_library(fake_ri):
    pm, _ = make_manager(fake_ri)
    library_wp = Waypoint.create('Library WP', ['j1', 'j2'], [1.0, 2.0])
    pm.add_existing_waypoint(library_wp)
    assert pm.program.elements[0].waypoint.name == 'Library WP'
    # confirms it's a copy, not a shared reference
    library_wp.name = 'mutated'
    assert pm.program.elements[0].waypoint.name == 'Library WP'


def test_rename_only_affects_waypoint_elements(fake_ri):
    pm, _ = make_manager(fake_ri)
    pm.add_waypoint()
    pm.add_jaw_open()
    pm.rename_element(0, 'MyWaypoint')
    assert pm.program.elements[0].waypoint.name == 'MyWaypoint'
    pm.rename_element(1, 'ignored')  # jaw element, no-op
    assert pm.program.elements[1].element_type == ElementType.JAW_OPEN


def test_delete_move_up_down(fake_ri):
    pm, _ = make_manager(fake_ri)
    pm.add_waypoint()      # 0
    pm.add_jaw_close()     # 1
    pm.add_jaw_open()      # 2

    pm.move_up(2)
    assert [e.element_type for e in pm.program.elements] == [
        ElementType.WAYPOINT, ElementType.JAW_OPEN, ElementType.JAW_CLOSE,
    ]
    pm.move_down(0)
    assert [e.element_type for e in pm.program.elements] == [
        ElementType.JAW_OPEN, ElementType.WAYPOINT, ElementType.JAW_CLOSE,
    ]
    pm.delete_element(0)
    assert len(pm.program.elements) == 2


def test_new_program_resets(fake_ri):
    pm, _ = make_manager(fake_ri)
    pm.add_waypoint()
    pm.new_program('Fresh')
    assert pm.program.name == 'Fresh'
    assert len(pm.program.elements) == 0


# -- persistence --

def test_save_load_round_trip(fake_ri, tmp_path):
    pm, _ = make_manager(fake_ri)
    pm.add_waypoint()
    pm.add_jaw_close()
    pm.set_name('MyProgram')
    path = os.path.join(tmp_path, 'p.json')

    pm.save(path)
    pm.new_program()
    assert len(pm.program.elements) == 0

    pm.load(path)
    assert pm.program.name == 'MyProgram'
    assert len(pm.program.elements) == 2


# -- execution --

def test_run_empty_program_reports_error_and_does_not_crash(fake_ri):
    pm, _ = make_manager(fake_ri)
    errors = []
    pm.error_occurred.connect(errors.append)
    pm.run()
    assert errors
    assert pm.execution_state == ExecutionState.IDLE


def test_run_executes_in_order_and_waits_for_each_step(fake_ri):
    pm, _ = make_manager(fake_ri)
    pm.add_waypoint()   # 0: waypoint (arm goal)
    pm.add_jaw_close()  # 1: gripper goal
    pm.add_jaw_open()   # 2: gripper goal

    pm.run()
    assert pm.execution_state == ExecutionState.RUNNING
    assert pm.current_index == 0
    assert pm.element_state(pm.program.elements[0]) == ElementState.RUNNING
    # nothing beyond element 0 has been sent yet
    assert len(fake_ri.arm_goal_calls) == 1
    assert len(fake_ri.gripper_goal_calls) == 0

    fake_ri.succeed_last_arm_goal()
    assert pm.element_state(pm.program.elements[0]) == ElementState.COMPLETED
    assert pm.current_index == 1
    assert len(fake_ri.gripper_goal_calls) == 1  # jaw_close sent now, not before

    fake_ri.gripper_goal_calls  # sanity
    request_id = fake_ri.gripper_goal_calls[-1][2]
    from so101_dashboard.utils.signals import ActionOutcome
    fake_ri.trajectory_result.emit(ActionOutcome(accepted=True, succeeded=True, message='ok', request_id=request_id))
    assert pm.element_state(pm.program.elements[1]) == ElementState.COMPLETED
    assert pm.current_index == 2
    assert len(fake_ri.gripper_goal_calls) == 2  # jaw_open sent

    request_id = fake_ri.gripper_goal_calls[-1][2]
    fake_ri.trajectory_result.emit(ActionOutcome(accepted=True, succeeded=True, message='ok', request_id=request_id))
    assert pm.execution_state == ExecutionState.FINISHED
    assert all(pm.element_state(e) == ElementState.COMPLETED for e in pm.program.elements)


def test_stop_halts_execution_and_cancels(fake_ri):
    pm, _ = make_manager(fake_ri)
    pm.add_waypoint()
    pm.add_jaw_close()
    pm.run()
    assert pm.execution_state == ExecutionState.RUNNING

    pm.stop()
    assert pm.execution_state == ExecutionState.STOPPED
    assert fake_ri.cancel_all_calls == 1
    assert pm.element_state(pm.program.elements[0]) == ElementState.PENDING

    # a late result arriving after stop must not resurrect execution
    fake_ri.succeed_last_arm_goal()
    assert pm.execution_state == ExecutionState.STOPPED
    assert len(fake_ri.gripper_goal_calls) == 0  # never reached element 1


def test_gripper_stall_is_not_treated_as_failure(fake_ri):
    # GripperActionController reports the goal ABORTED (accepted=True,
    # succeeded=False) whenever the gripper stalls before reaching the
    # commanded position within tolerance -- expected when closing onto an
    # object, or when gripper_closed_position is set past the mechanical
    # limit on purpose. That must complete the element, not fail the
    # program, unlike an arm waypoint failing to reach its target.
    from so101_dashboard.utils.signals import ActionOutcome

    pm, _ = make_manager(fake_ri)
    pm.add_jaw_close()
    pm.add_jaw_open()
    errors = []
    pm.error_occurred.connect(errors.append)

    pm.run()
    request_id = fake_ri.gripper_goal_calls[-1][2]
    fake_ri.trajectory_result.emit(
        ActionOutcome(accepted=True, succeeded=False, message='Goal finished with status 6', request_id=request_id)
    )

    assert pm.element_state(pm.program.elements[0]) == ElementState.COMPLETED
    assert pm.current_index == 1
    assert pm.execution_state == ExecutionState.RUNNING
    assert not errors  # no "Program stopped" popup for a gripper stall


def test_arm_goal_not_succeeded_still_fails_program(fake_ri):
    # The gripper-stall exception must NOT apply to waypoint (arm) elements
    # -- an arm goal that doesn't report success is a real problem.
    pm, _ = make_manager(fake_ri)
    pm.add_waypoint()
    errors = []
    pm.error_occurred.connect(errors.append)

    pm.run()
    from so101_dashboard.utils.signals import ActionOutcome
    request_id = fake_ri.arm_goal_calls[-1][3]
    fake_ri.trajectory_result.emit(
        ActionOutcome(accepted=True, succeeded=False, message='Goal finished with status 6', request_id=request_id)
    )

    assert pm.execution_state == ExecutionState.FAILED
    assert errors


def test_rejected_goal_fails_program_and_stops_advancing(fake_ri):
    pm, _ = make_manager(fake_ri)
    pm.add_waypoint()
    pm.add_jaw_close()
    errors = []
    pm.error_occurred.connect(errors.append)

    pm.run()
    fake_ri.reject_last_arm_goal('busy')

    assert pm.execution_state == ExecutionState.FAILED
    assert pm.element_state(pm.program.elements[0]) == ElementState.FAILED
    assert errors
    assert len(fake_ri.gripper_goal_calls) == 0  # never advanced to element 1


def test_out_of_limits_waypoint_fails_without_sending_goal(fake_ri):
    pm, _ = make_manager(fake_ri)
    pm.add_waypoint()
    pm.program.elements[0].waypoint.joint_positions[0] = 99.0  # out of limits

    errors = []
    pm.error_occurred.connect(errors.append)
    pm.run()

    assert pm.execution_state == ExecutionState.FAILED
    assert len(fake_ri.arm_goal_calls) == 0
    assert errors


# -- multi-program registry --

def test_new_program_adds_rather_than_replaces(fake_ri):
    pm, _ = make_manager(fake_ri)
    pm.add_waypoint()
    pm.set_name('First')
    pm.new_program('Second')

    assert set(pm.program_names) == {'First', 'Second'}
    assert pm.active_name == 'Second'
    assert len(pm.program.elements) == 0
    # switching back must not have lost the first program's elements
    pm.select_program('First')
    assert len(pm.program.elements) == 1


def test_new_program_deduplicates_name(fake_ri):
    pm, _ = make_manager(fake_ri)
    pm.new_program('Dup')
    pm.new_program('Dup')
    assert 'Dup' in pm.program_names
    assert 'Dup (2)' in pm.program_names


def test_select_program_switches_active_without_running(fake_ri):
    pm, _ = make_manager(fake_ri)
    pm.set_name('A')
    pm.new_program('B')
    pm.add_jaw_open()  # only affects B

    assert pm.select_program('A') is True
    assert pm.active_name == 'A'
    assert len(pm.program.elements) == 0
    pm.select_program('B')
    assert len(pm.program.elements) == 1


def test_select_program_blocked_while_running(fake_ri):
    pm, _ = make_manager(fake_ri)
    pm.add_waypoint()      # recorded into 'Untitled'
    pm.new_program('B')    # active becomes 'B', empty
    pm.select_program('Untitled')  # back to the one with content
    pm.run()

    errors = []
    pm.error_occurred.connect(errors.append)
    assert pm.select_program('B') is False
    assert errors


def test_rename_program_updates_registry_key(fake_ri):
    pm, _ = make_manager(fake_ri)
    pm.add_waypoint()
    assert pm.rename_program('Renamed') is True
    assert pm.active_name == 'Renamed'
    assert 'Renamed' in pm.program_names
    assert len(pm.program.elements) == 1


def test_rename_program_rejects_collision(fake_ri):
    pm, _ = make_manager(fake_ri)
    pm.set_name('A')
    pm.new_program('B')
    errors = []
    pm.error_occurred.connect(errors.append)
    assert pm.rename_program('A') is False
    assert errors


def test_delete_program_falls_back_to_remaining_program(fake_ri):
    pm, _ = make_manager(fake_ri)
    pm.set_name('A')
    pm.new_program('B')
    assert pm.delete_program('B') is True
    assert 'B' not in pm.program_names
    assert pm.active_name == 'A'


def test_delete_last_program_leaves_a_fresh_untitled(fake_ri):
    pm, _ = make_manager(fake_ri)
    pm.set_name('OnlyOne')
    pm.delete_program('OnlyOne')
    assert pm.program_names == ['Untitled']
    assert pm.active_name == 'Untitled'
    assert len(pm.program.elements) == 0


def test_delete_active_program_while_running_is_blocked(fake_ri):
    pm, _ = make_manager(fake_ri)
    pm.add_waypoint()
    pm.run()
    errors = []
    pm.error_occurred.connect(errors.append)
    assert pm.delete_program(pm.active_name) is False
    assert errors


def test_load_all_from_directory_populates_registry(fake_ri, tmp_path):
    pm, _ = make_manager(fake_ri)
    pm.add_waypoint()
    pm.set_name('OnDisk1')
    pm.save(os.path.join(tmp_path, 'OnDisk1.json'))
    pm.new_program('OnDisk2')
    pm.add_jaw_open()
    pm.save(os.path.join(tmp_path, 'OnDisk2.json'))

    fresh, _ = make_manager(fake_ri)
    fresh.load_all_from_directory(str(tmp_path))
    assert set(fresh.program_names) == {'OnDisk1', 'OnDisk2'}
    assert 'Untitled' not in fresh.program_names  # empty placeholder dropped


# -- per-element velocity/acceleration scaling --

def test_new_waypoint_element_scaling_defaults_to_none(fake_ri):
    pm, _ = make_manager(fake_ri)
    pm.add_waypoint()
    el = pm.program.elements[0]
    assert el.velocity_scaling is None
    assert el.acceleration_scaling is None


def test_resolved_scaling_falls_back_to_configured_default(fake_ri):
    pm, _ = make_manager(fake_ri)
    pm.add_waypoint()
    el = pm.program.elements[0]
    vel, acc = pm.resolved_scaling(el)
    assert vel == 0.3  # default_velocity_scaling fallback in ProgramManager
    assert acc == 0.2  # default_acceleration_scaling fallback


def test_set_element_scaling_overrides_default(fake_ri):
    pm, _ = make_manager(fake_ri)
    pm.add_waypoint()
    pm.set_element_scaling(0, 0.9, 0.8)
    vel, acc = pm.resolved_scaling(pm.program.elements[0])
    assert vel == 0.9
    assert acc == 0.8


def test_set_element_scaling_none_resets_to_default(fake_ri):
    pm, _ = make_manager(fake_ri)
    pm.add_waypoint()
    pm.set_element_scaling(0, 0.9, 0.8)
    pm.set_element_scaling(0, None, None)
    vel, acc = pm.resolved_scaling(pm.program.elements[0])
    assert vel == 0.3
    assert acc == 0.2


def test_set_element_scaling_ignored_for_non_waypoint_elements(fake_ri):
    pm, _ = make_manager(fake_ri)
    pm.add_jaw_open()
    pm.set_element_scaling(0, 0.9, 0.8)  # no-op, no crash
    assert pm.program.elements[0].velocity_scaling is None


def test_per_element_velocity_scaling_changes_executed_move_duration(fake_ri):
    # A waypoint given a much higher velocity_scaling than the configured
    # default must produce a shorter commanded duration -- proves the
    # executor actually reads the per-element override, not just the flat
    # program_waypoint_duration.
    pm, _ = make_manager(fake_ri)
    pm.add_waypoint()  # default scaling -> slower
    pm.add_waypoint()  # will override -> faster
    pm.set_element_scaling(1, 1.0, 1.0)

    pm.run()
    _, _, default_duration, _ = fake_ri.arm_goal_calls[0]
    fake_ri.succeed_last_arm_goal()
    _, _, fast_duration, _ = fake_ri.arm_goal_calls[1]

    assert fast_duration < default_duration


# -- looping --

def test_default_loop_count_is_one_and_run_finishes_after_one_pass(fake_ri):
    pm, _ = make_manager(fake_ri)
    pm.add_waypoint()
    pm.run()
    fake_ri.succeed_last_arm_goal()
    assert pm.execution_state == ExecutionState.FINISHED
    assert pm.current_loop == 1


def test_set_loop_count_is_strictly_binary(fake_ri):
    # loop_count only ever means "forever" (0) or "once" (anything else)
    # -- there's no "run N times" anymore, so any nonzero input collapses
    # to 1, not to whatever N was passed in. This is what actually fixes
    # the "why is there 3/3" confusion: it's not just that the UI only
    # offers a checkbox, the manager itself can no longer be made to hold
    # an arbitrary count.
    pm, _ = make_manager(fake_ri)
    pm.set_loop_count(5)
    assert pm.program.loop_count == 1
    pm.set_loop_count(-5)
    assert pm.program.loop_count == 1
    pm.set_loop_count(0)
    assert pm.program.loop_count == 0


def test_set_loop_count_blocked_while_running(fake_ri):
    pm, _ = make_manager(fake_ri)
    pm.add_waypoint()
    pm.set_loop_count(0)
    pm.run()
    errors = []
    pm.error_occurred.connect(errors.append)
    pm.set_loop_count(1)
    assert pm.program.loop_count == 0  # unchanged
    assert errors


def test_loading_a_legacy_program_with_an_arbitrary_loop_count_normalizes_it(fake_ri, tmp_path):
    # A program saved by an older version of the app (back when loop count
    # was a free-form spinbox) could have loop_count: 3 on disk. Loading
    # it must not resurrect "run 3 times" or a "Loop 3/3" display -- it
    # collapses to a normal single run, same as anything else nonzero.
    import json
    path = os.path.join(tmp_path, 'legacy.json')
    with open(path, 'w') as f:
        json.dump({'program_name': 'Legacy', 'version': 1, 'loop_count': 3, 'elements': []}, f)

    pm, _ = make_manager(fake_ri)
    pm.load(path)
    assert pm.program.loop_count == 1


def test_loop_count_zero_runs_until_stopped(fake_ri):
    pm, _ = make_manager(fake_ri)
    pm.add_waypoint()
    pm.set_loop_count(0)

    pm.run()
    for _ in range(10):
        assert pm.execution_state == ExecutionState.RUNNING
        fake_ri.succeed_last_arm_goal()
    assert pm.current_loop == 11  # kept going well past any finite loop_count
    assert pm.execution_state == ExecutionState.RUNNING

    pm.stop()
    assert pm.execution_state == ExecutionState.STOPPED
    # a late result after STOP must not resurrect the loop
    fake_ri.succeed_last_arm_goal()
    assert pm.execution_state == ExecutionState.STOPPED


def test_loop_progress_signal_reports_current_and_total(fake_ri):
    pm, _ = make_manager(fake_ri)
    pm.add_waypoint()
    pm.set_loop_count(0)  # forever
    progress = []
    pm.loop_progress_changed.connect(lambda cur, total: progress.append((cur, total)))

    pm.run()
    assert progress[-1] == (1, 0)
    fake_ri.succeed_last_arm_goal()
    assert progress[-1] == (2, 0)


def test_failure_during_a_loop_stops_the_whole_program_not_just_that_iteration(fake_ri):
    pm, _ = make_manager(fake_ri)
    pm.add_waypoint()
    pm.set_loop_count(0)  # infinite -- failure must still win
    pm.run()
    fake_ri.reject_last_arm_goal('busy')
    assert pm.execution_state == ExecutionState.FAILED
    assert pm.current_loop == 1


def test_loop_count_persists_per_program_across_switches(fake_ri):
    pm, _ = make_manager(fake_ri)
    pm.set_name('A')
    pm.set_loop_count(0)  # forever
    pm.new_program('B')
    assert pm.program.loop_count == 1  # fresh program, unaffected
    pm.select_program('A')
    assert pm.program.loop_count == 0


def test_cannot_edit_while_running(fake_ri):
    pm, _ = make_manager(fake_ri)
    pm.add_waypoint()
    pm.add_jaw_close()
    pm.run()

    errors = []
    pm.error_occurred.connect(errors.append)
    pm.add_jaw_open()
    pm.delete_element(0)

    assert len(pm.program.elements) == 2  # unchanged
    assert len(errors) == 2


# -- re-recording a waypoint element in place --

def test_re_record_waypoint_overwrites_positions_keeps_name_and_slot(fake_ri):
    pm, rs = make_manager(fake_ri, positions={'j1': 0.1, 'j2': 0.2, 'gripper': 0.3})
    pm.add_waypoint()
    pm.add_jaw_close()
    pm.add_waypoint()
    original_name = pm.program.elements[0].waypoint.name

    rs.positions = {'j1': 0.9, 'j2': 0.8, 'gripper': 0.7}
    assert pm.re_record_waypoint(0) is True

    el = pm.program.elements[0]
    assert el.waypoint.name == original_name  # unchanged
    assert el.waypoint.joint_positions == [0.9, 0.8, 0.7]
    # sequence position and other elements untouched
    assert len(pm.program.elements) == 3
    assert pm.program.elements[1].element_type == ElementType.JAW_CLOSE


def test_re_record_waypoint_fails_cleanly_if_data_missing(fake_ri):
    pm, rs = make_manager(fake_ri)
    pm.add_waypoint()
    rs.positions = {'j1': 0.1}  # j2/gripper missing now
    errors = []
    pm.error_occurred.connect(errors.append)

    assert pm.re_record_waypoint(0) is False
    assert errors
    assert pm.program.elements[0].waypoint.joint_positions == [0.1, 0.2, 0.3]  # unchanged


def test_re_record_rejects_non_waypoint_element(fake_ri):
    pm, _ = make_manager(fake_ri)
    pm.add_jaw_open()
    errors = []
    pm.error_occurred.connect(errors.append)
    assert pm.re_record_waypoint(0) is False
    assert errors


def test_re_record_blocked_while_running(fake_ri):
    pm, _ = make_manager(fake_ri)
    pm.add_waypoint()
    pm.add_jaw_close()
    pm.run()
    errors = []
    pm.error_occurred.connect(errors.append)
    assert pm.re_record_waypoint(0) is False
    assert errors


# -- executing a single element without running the whole program --

def test_execute_element_sends_only_that_elements_goal(fake_ri):
    pm, _ = make_manager(fake_ri)
    pm.add_waypoint()   # 0
    pm.add_jaw_close()  # 1
    pm.add_jaw_open()   # 2

    assert pm.execute_element(1) is True
    assert len(fake_ri.gripper_goal_calls) == 1
    assert len(fake_ri.arm_goal_calls) == 0
    assert pm.execution_state == ExecutionState.RUNNING
    assert pm.element_state(pm.program.elements[1]) == ElementState.RUNNING
    # untouched elements stay untouched
    assert pm.element_state(pm.program.elements[0]) == ElementState.PENDING
    assert pm.element_state(pm.program.elements[2]) == ElementState.PENDING


def test_execute_element_completes_to_idle_without_advancing_to_next(fake_ri):
    pm, _ = make_manager(fake_ri)
    pm.add_waypoint()   # 0
    pm.add_jaw_close()  # 1

    pm.execute_element(0)
    fake_ri.succeed_last_arm_goal()

    assert pm.execution_state == ExecutionState.IDLE
    assert pm.element_state(pm.program.elements[0]) == ElementState.COMPLETED
    assert len(fake_ri.gripper_goal_calls) == 0  # never touched element 1


def test_execute_element_failure_reported_as_element_failure_not_program_stopped(fake_ri):
    pm, _ = make_manager(fake_ri)
    pm.add_waypoint()
    errors = []
    pm.error_occurred.connect(errors.append)

    pm.execute_element(0)
    fake_ri.reject_last_arm_goal('busy')

    assert pm.execution_state == ExecutionState.FAILED
    assert pm.element_state(pm.program.elements[0]) == ElementState.FAILED
    assert errors
    assert 'Element execution failed' in errors[0]


def test_execute_element_blocked_while_program_is_running(fake_ri):
    pm, _ = make_manager(fake_ri)
    pm.add_waypoint()
    pm.add_jaw_close()
    pm.run()

    errors = []
    pm.error_occurred.connect(errors.append)
    assert pm.execute_element(1) is False
    assert errors


def test_stop_cancels_a_single_element_execution(fake_ri):
    pm, _ = make_manager(fake_ri)
    pm.add_waypoint()
    pm.execute_element(0)
    assert pm.execution_state == ExecutionState.RUNNING

    pm.stop()
    assert pm.execution_state == ExecutionState.STOPPED
    assert pm.element_state(pm.program.elements[0]) == ElementState.PENDING
    assert fake_ri.cancel_all_calls == 1

    # a late result after STOP must not resurrect it or advance anything
    fake_ri.succeed_last_arm_goal()
    assert pm.execution_state == ExecutionState.STOPPED


# -- duplicating an element in place --

def test_duplicate_waypoint_inserts_copy_right_after_with_copy_suffix(fake_ri):
    pm, _ = make_manager(fake_ri)
    pm.add_waypoint()   # 0
    pm.add_jaw_close()  # 1
    original_name = pm.program.elements[0].waypoint.name
    original_positions = list(pm.program.elements[0].waypoint.joint_positions)

    assert pm.duplicate_element(0) is True

    assert len(pm.program.elements) == 3
    dup = pm.program.elements[1]
    assert dup.element_type == ElementType.WAYPOINT
    assert dup.waypoint.name == f'{original_name}_copy'
    assert dup.waypoint.joint_positions == original_positions
    assert dup.element_id != pm.program.elements[0].element_id  # distinct element
    # original and the jaw_close element are untouched, just shifted
    assert pm.program.elements[0].waypoint.name == original_name
    assert pm.program.elements[2].element_type == ElementType.JAW_CLOSE


def test_duplicate_waypoint_preserves_speed_override(fake_ri):
    pm, _ = make_manager(fake_ri)
    pm.add_waypoint()
    pm.set_element_scaling(0, 0.8, 0.7)

    pm.duplicate_element(0)

    dup = pm.program.elements[1]
    assert dup.velocity_scaling == 0.8
    assert dup.acceleration_scaling == 0.7


def test_duplicate_jaw_element(fake_ri):
    pm, _ = make_manager(fake_ri)
    pm.add_jaw_open()
    assert pm.duplicate_element(0) is True
    assert len(pm.program.elements) == 2
    assert pm.program.elements[1].element_type == ElementType.JAW_OPEN


def test_duplicate_element_blocked_while_running(fake_ri):
    pm, _ = make_manager(fake_ri)
    pm.add_waypoint()
    pm.add_jaw_close()
    pm.run()
    errors = []
    pm.error_occurred.connect(errors.append)
    assert pm.duplicate_element(0) is False
    assert len(pm.program.elements) == 2
    assert errors


def test_duplicate_element_invalid_index_is_noop(fake_ri):
    pm, _ = make_manager(fake_ri)
    pm.add_waypoint()
    assert pm.duplicate_element(5) is False
    assert len(pm.program.elements) == 1


# -- wait/delay elements --

def test_add_wait_inserts_element_with_given_duration(fake_ri):
    pm, _ = make_manager(fake_ri)
    pm.add_wait(duration_sec=2.5)
    assert len(pm.program.elements) == 1
    el = pm.program.elements[0]
    assert el.element_type == ElementType.WAIT
    assert el.wait_duration_sec == 2.5


def test_add_wait_defaults_to_one_second(fake_ri):
    pm, _ = make_manager(fake_ri)
    pm.add_wait()
    assert pm.program.elements[0].wait_duration_sec == 1.0


def test_add_wait_clamps_negative_duration(fake_ri):
    pm, _ = make_manager(fake_ri)
    pm.add_wait(duration_sec=-3.0)
    assert pm.program.elements[0].wait_duration_sec == 0.0


def test_set_wait_duration_updates_existing_wait_element(fake_ri):
    pm, _ = make_manager(fake_ri)
    pm.add_wait(duration_sec=1.0)
    pm.set_wait_duration(0, 4.0)
    assert pm.program.elements[0].wait_duration_sec == 4.0


def test_set_wait_duration_ignored_for_non_wait_elements(fake_ri):
    pm, _ = make_manager(fake_ri)
    pm.add_jaw_open()
    pm.set_wait_duration(0, 4.0)  # no-op, no crash
    assert pm.program.elements[0].element_type == ElementType.JAW_OPEN


def test_duplicate_wait_element_preserves_duration(fake_ri):
    pm, _ = make_manager(fake_ri)
    pm.add_wait(duration_sec=3.0)
    pm.duplicate_element(0)
    assert len(pm.program.elements) == 2
    assert pm.program.elements[1].element_type == ElementType.WAIT
    assert pm.program.elements[1].wait_duration_sec == 3.0


def test_wait_element_sends_no_goal_and_completes_via_timer(fake_ri):
    pm, _ = make_manager(fake_ri)
    pm.add_waypoint()   # 0
    pm.add_wait(duration_sec=2.0)  # 1
    pm.add_jaw_close()  # 2

    pm.run()
    fake_ri.succeed_last_arm_goal()  # element 0 done -> advances into the wait

    assert pm.execution_state == ExecutionState.RUNNING
    assert pm.current_index == 1
    assert pm.element_state(pm.program.elements[1]) == ElementState.RUNNING
    assert len(fake_ri.arm_goal_calls) == 1  # the wait itself sent nothing
    assert len(fake_ri.gripper_goal_calls) == 0  # hasn't reached jaw_close yet

    pm._wait_timer.timeout.emit()  # simulate the wait duration elapsing

    assert pm.element_state(pm.program.elements[1]) == ElementState.COMPLETED
    assert pm.current_index == 2
    assert len(fake_ri.gripper_goal_calls) == 1  # advanced into jaw_close


def test_stop_during_a_wait_cancels_it_and_late_timer_is_a_noop(fake_ri):
    pm, _ = make_manager(fake_ri)
    pm.add_wait(duration_sec=5.0)
    pm.run()
    assert pm.execution_state == ExecutionState.RUNNING
    wait_timer = pm._wait_timer

    pm.stop()
    assert pm.execution_state == ExecutionState.STOPPED
    assert pm.element_state(pm.program.elements[0]) == ElementState.PENDING

    wait_timer.timeout.emit()  # late fire after STOP must not resurrect anything
    assert pm.execution_state == ExecutionState.STOPPED
    assert pm.element_state(pm.program.elements[0]) == ElementState.PENDING


def test_execute_element_on_a_wait_completes_to_idle_without_advancing(fake_ri):
    pm, _ = make_manager(fake_ri)
    pm.add_wait(duration_sec=1.0)  # 0
    pm.add_jaw_close()             # 1

    pm.execute_element(0)
    assert pm.execution_state == ExecutionState.RUNNING
    pm._wait_timer.timeout.emit()

    assert pm.execution_state == ExecutionState.IDLE
    assert pm.element_state(pm.program.elements[0]) == ElementState.COMPLETED
    assert len(fake_ri.gripper_goal_calls) == 0  # never touched element 1


def test_a_stale_wait_timer_from_a_replaced_execution_is_ignored(fake_ri):
    # If somehow a wait's timer instance survives (e.g. a slow real
    # QTimer) past the point its execution ended and a NEW one began, the
    # stale one's request_id must not match the new one's -- guards the
    # request_id gate itself, not just the execution_state gate.
    pm, _ = make_manager(fake_ri)
    pm.add_wait(duration_sec=1.0)
    pm.execute_element(0)
    stale_timer = pm._wait_timer
    stale_request_id = pm._current_request_id

    pm._on_wait_finished(stale_request_id)  # legitimately completes it
    assert pm.execution_state == ExecutionState.IDLE

    pm.execute_element(0)  # a fresh execution, new request_id
    assert pm._current_request_id != stale_request_id

    pm._on_wait_finished(stale_request_id)  # late/duplicate call with the OLD id
    assert pm.execution_state == ExecutionState.RUNNING  # untouched by the stale call
