"""Unit tests for JointControlManager: target validation (never silently
clamps), the coordinated multi-joint stream (arm_controller requires every
joint in every goal), the gripper's independent stream, throttled/coalesced
sending, and HOME reuse.

Not gated on torque state (removed by request) -- streaming/go_home()
always send, regardless of TorqueManager. This module has no
TorqueManager dependency at all.

No explicit Apply step (removed by request): set_target()/nudge() send
immediately on the leading edge (first change after being idle) and
throttle any further changes to at most one goal per throttle interval,
via two independent QTimers (arm, gripper). Since these tests don't run a
real Qt event loop, throttled ticks are simulated by calling the timer's
timeout handler directly (_on_arm_send_timer_timeout /
_on_gripper_send_timer_timeout) rather than waiting on a real QTimer.
"""
from so101_dashboard.robot.joint_control_manager import JointControlManager
from so101_dashboard.robot.joint_limits import JointLimit, JointLimits

ARM_JOINTS = ['j1', 'j2']
GRIPPER = 'gripper'
HOME_POSITION = {'j1': 0.0, 'j2': 0.1}
MOTION_CONFIG = {
    'joint_control_stream_duration': 0.2,
    'joint_control_stream_interval_ms': 50,
    'gripper_hold_max_effort': 50.0,
}


class FakeRobotState:
    def __init__(self, positions):
        self.positions = positions
        self.joint_state_updated = _NullSignal()


class _NullSignal:
    """JointControlManager connects to robot_state.joint_state_updated;
    for these tests we drive positions directly and don't need it to fire."""
    def connect(self, *_args, **_kwargs):
        pass


def make_limits():
    return JointLimits({
        'j1': JointLimit(min_position=-1.0, max_position=1.0),
        'j2': JointLimit(min_position=-1.0, max_position=1.0),
        'gripper': JointLimit(min_position=-0.2, max_position=1.6),
    })


def make_manager(fake_ri, positions=None):
    rs = FakeRobotState(positions or {'j1': 0.1, 'j2': 0.2, 'gripper': 0.3})
    jcm = JointControlManager(
        fake_ri, rs, make_limits(), ARM_JOINTS, GRIPPER,
        HOME_POSITION, 4.0, MOTION_CONFIG,
    )
    return jcm, rs


# -- target editing / validation --

def test_set_target_accepts_valid_value(fake_ri):
    jcm, _ = make_manager(fake_ri)
    assert jcm.set_target('j1', 0.5) is True
    assert jcm.target('j1') == 0.5


def test_set_target_rejects_non_finite(fake_ri):
    jcm, _ = make_manager(fake_ri)
    jcm.set_target('j1', 0.5)
    errors = []
    jcm.error_occurred.connect(errors.append)

    assert jcm.set_target('j1', float('nan')) is False
    assert jcm.target('j1') == 0.5  # unchanged, not silently clamped
    assert errors


def test_set_target_rejects_out_of_range_without_clamping(fake_ri):
    jcm, _ = make_manager(fake_ri)
    jcm.set_target('j1', 0.5)
    errors = []
    jcm.error_occurred.connect(errors.append)

    assert jcm.set_target('j1', 99.0) is False
    assert jcm.target('j1') == 0.5  # unchanged
    assert errors and 'j1' in errors[0]


def test_rejected_target_sends_nothing(fake_ri):
    jcm, _ = make_manager(fake_ri)
    jcm.set_target('j1', 99.0)  # rejected -- out of range
    assert len(fake_ri.arm_goal_calls) == 0


def test_nudge_adjusts_from_current_target(fake_ri):
    jcm, _ = make_manager(fake_ri)
    jcm.set_target('j1', 0.5)
    assert jcm.nudge('j1', 0.1) is True
    assert abs(jcm.target('j1') - 0.6) < 1e-9


def test_nudge_falls_back_to_current_actual_if_no_target_set(fake_ri):
    jcm, _ = make_manager(fake_ri, positions={'j1': 0.2, 'j2': 0.0, 'gripper': 0.0})
    assert jcm.nudge('j1', 0.1) is True
    assert abs(jcm.target('j1') - 0.3) < 1e-9


def test_nudge_also_streams_immediately(fake_ri):
    jcm, _ = make_manager(fake_ri, positions={'j1': 0.2, 'j2': 0.0, 'gripper': 0.0})
    jcm.nudge('j1', 0.1)
    assert len(fake_ri.arm_goal_calls) == 1


# -- regression: untouched joints must never go stale --

def test_untouched_joint_target_tracks_live_position(fake_ri):
    # Bug this guards against: a joint's target used to be set ONCE (first
    # time seen) and never again, so if the robot moved via anything else
    # (a Program, a waypoint, HOME, manual backdrive) BEFORE it was ever
    # seeded, it must pick up wherever the robot actually is -- not
    # whatever arbitrary value happened to be in RobotState first.
    rs_positions = {'j1': 0.1, 'j2': 0.2, 'gripper': 0.3}
    jcm, rs = make_manager(fake_ri, positions=rs_positions)
    jcm._sync_untouched_targets()  # simulates the first real /joint_states arriving
    assert jcm.target('j2') == 0.2

    # robot moves via something unrelated to this joint's control (e.g. a
    # Program run, or manual backdrive) -- j2 was never explicitly touched.
    # sync_to_current() (tab re-show, or the button) is what's meant to
    # pick this up now, not another passive _sync_untouched_targets() tick
    # -- see test_untouched_joint_is_seeded_once_not_continuously below
    # for why that changed.
    rs_positions['j2'] = 0.9
    jcm.sync_to_current()
    assert jcm.target('j2') == 0.9, "sync_to_current() must follow live position, not stay stale"


def test_untouched_joint_is_seeded_once_not_continuously(fake_ri):
    # Regression guard for the real-hardware jitter bug: continuously
    # re-syncing an untouched joint's target to the live (noisy, laggy)
    # sensor reading on every single tick -- while it's being repeatedly
    # re-sent as part of an unrelated joint's active streaming -- turned
    # into a feedback loop that made every "held" joint visibly twitch.
    # Only the FIRST tick after being unseeded may change the target;
    # every tick after that must leave it alone until touched or
    # explicitly resynced.
    rs_positions = {'j1': 0.1, 'j2': 0.2, 'gripper': 0.3}
    jcm, rs = make_manager(fake_ri, positions=rs_positions)
    jcm._sync_untouched_targets()  # seeds j2 to 0.2
    assert jcm.target('j2') == 0.2

    rs_positions['j2'] = 0.9  # sensor reading drifts (noise/latency), joint never touched
    jcm._sync_untouched_targets()
    assert jcm.target('j2') == 0.2, 'must NOT keep chasing live position tick after tick'

    rs_positions['j2'] = -0.4
    jcm._sync_untouched_targets()
    assert jcm.target('j2') == 0.2, 'still must not have moved -- only touched/sync_to_current can change it'


def test_touched_joint_target_stops_following_live_position(fake_ri):
    rs_positions = {'j1': 0.1, 'j2': 0.2, 'gripper': 0.3}
    jcm, rs = make_manager(fake_ri, positions=rs_positions)
    jcm.set_target('j2', 0.5)  # user deliberately sets a target

    rs_positions['j2'] = 0.9  # robot's actual position drifts elsewhere
    jcm._sync_untouched_targets()
    assert jcm.target('j2') == 0.5, "user's chosen target must not be overwritten"


def test_stream_never_sends_stale_value_for_untouched_joint(fake_ri):
    rs_positions = {'j1': 0.1, 'j2': 0.2, 'gripper': 0.3}
    jcm, rs = make_manager(fake_ri, positions=rs_positions)

    # time passes, j2 moves via something else entirely, before the user
    # ever touches Joint Control for j2
    rs_positions['j2'] = 0.9
    jcm._sync_untouched_targets()

    jcm.set_target('j1', 0.6)  # user only adjusts j1 -- this alone sends

    names, positions, _, _ = fake_ri.arm_goal_calls[0]
    assert dict(zip(names, positions))['j2'] == 0.9, 'must send j2 at its current position, not the stale 0.2'


def test_held_joint_sent_value_stays_constant_across_a_drag_despite_sensor_noise(fake_ri):
    # End-to-end reproduction of the reported real-hardware bug: drag j1
    # continuously (leading-edge send, then a throttle tick, then
    # another), with j2's live sensor reading drifting slightly on every
    # /joint_states tick in between (as real feedback does -- latency,
    # encoder noise). j2 was never touched. Every goal actually SENT for
    # j2 must be identical -- if it isn't, j2 is being asked to make a
    # fresh tiny "correction" every cycle, which is exactly what made
    # every other joint visibly jitter while dragging just one.
    rs_positions = {'j1': 0.1, 'j2': 0.2, 'gripper': 0.3}
    jcm, rs = make_manager(fake_ri, positions=rs_positions)
    jcm._sync_untouched_targets()  # seed, as if the tab was just opened

    jcm.set_target('j1', 0.3)  # leading-edge send #1
    rs_positions['j2'] = 0.201  # tiny sensor drift/noise
    jcm._sync_untouched_targets()

    jcm._on_arm_send_timer_timeout()  # nothing changed -> timer goes idle, no send
    jcm.set_target('j1', 0.4)  # leading-edge send #2 (timer had gone idle)
    rs_positions['j2'] = 0.199  # noise wobbles the other way
    jcm._sync_untouched_targets()

    jcm._on_arm_send_timer_timeout()
    jcm.set_target('j1', 0.5)  # leading-edge send #3
    rs_positions['j2'] = 0.203
    jcm._sync_untouched_targets()

    j2_values_sent = [dict(zip(names, positions))['j2'] for names, positions, _, _ in fake_ri.arm_goal_calls]
    assert len(j2_values_sent) == 3
    assert all(v == j2_values_sent[0] for v in j2_values_sent), (
        f"j2 was re-sent a different value every cycle despite never being touched: {j2_values_sent}"
    )


# -- regression: a TOUCHED joint must not stay stale forever either, once
# the robot has moved elsewhere (a Program, a Waypoint, manual backdrive)
# and the user comes back to jog a different joint -- sync_to_current()
# (called automatically on tab-show, and manually via the button) is what
# breaks a touched joint's permanent lock.

def test_sync_to_current_clears_touched_and_resyncs_all_targets(fake_ri):
    rs_positions = {'j1': 0.1, 'j2': 0.2, 'gripper': 0.3}
    jcm, rs = make_manager(fake_ri, positions=rs_positions)
    jcm.set_target('j1', 0.9)  # j1 touched, frozen at 0.9

    # robot moves elsewhere entirely (e.g. a Program ran) while Joint
    # Control wasn't driving it -- j1's stale touched target never got a
    # chance to follow along, unlike an untouched joint would have
    rs_positions['j1'] = -0.4
    rs_positions['j2'] = 0.7

    jcm.sync_to_current()

    assert jcm.target('j1') == -0.4, "touched joint's stale lock must be released"
    assert jcm.target('j2') == 0.7


def test_touched_joint_snaps_back_without_sync_reproduces_the_bug(fake_ri):
    # Demonstrates the actual bug being fixed: WITHOUT sync_to_current(),
    # a previously-touched joint keeps re-sending its old value forever,
    # even after the robot has moved elsewhere and the user only meant to
    # jog a different joint.
    rs_positions = {'j1': 0.1, 'j2': 0.2, 'gripper': 0.3}
    jcm, rs = make_manager(fake_ri, positions=rs_positions)
    jcm.set_target('j1', 0.9)  # j1 touched

    rs_positions['j1'] = -0.4  # robot moved elsewhere; j1 never re-synced
    jcm.set_target('j2', 0.6)  # user only means to move j2

    names, positions, _, _ = fake_ri.arm_goal_calls[-1]
    by_name = dict(zip(names, positions))
    assert by_name['j1'] == 0.9, 'without sync, j1 snaps back to its old touched value -- this is the bug'


def test_sync_to_current_fixes_the_stale_touched_joint_bug(fake_ri):
    rs_positions = {'j1': 0.1, 'j2': 0.2, 'gripper': 0.3}
    jcm, rs = make_manager(fake_ri, positions=rs_positions)
    jcm.set_target('j1', 0.9)  # j1 touched -- leading-edge send, starts the throttle window

    rs_positions['j1'] = -0.4  # robot moved elsewhere
    jcm.sync_to_current()      # e.g. Joint Control tab re-shown, or button pressed

    jcm.set_target('j2', 0.6)  # user only means to move j2 -- coalesced, still inside the window
    jcm._on_arm_send_timer_timeout()  # simulate the throttle interval elapsing, flush it

    names, positions, _, _ = fake_ri.arm_goal_calls[-1]
    by_name = dict(zip(names, positions))
    assert by_name['j1'] == -0.4, "j1 must hold where the robot actually is, not snap back"


def test_sync_to_current_emits_targets_changed_even_with_no_touched_joints(fake_ri):
    jcm, _ = make_manager(fake_ri)
    changed = []
    jcm.targets_changed.connect(lambda: changed.append(True))
    jcm.sync_to_current()
    assert changed


# -- streaming is unconditional (no torque gating) --

def test_stream_sends_regardless_of_anything_torque_related(fake_ri):
    jcm, _ = make_manager(fake_ri)
    jcm.set_target('j1', 0.5)
    assert len(fake_ri.arm_goal_calls) == 1


def test_joint_control_manager_never_touches_torque_service(fake_ri):
    # No TorqueManager dependency at all -- confirm nothing torque-related
    # was ever invoked on the fake even after normal use.
    jcm, _ = make_manager(fake_ri)
    jcm.set_target('j1', 0.5)
    jcm.set_target('gripper', 1.0)
    jcm.go_home()
    assert fake_ri.toggle_torque_calls == []


# -- coordinated stream (arm_controller needs all joints every goal) --

def test_stream_sends_all_arm_joints_using_targets_and_fallback_to_current(fake_ri):
    jcm, _ = make_manager(fake_ri, positions={'j1': 0.05, 'j2': 0.2, 'gripper': 0.0})
    jcm.set_target('j1', 0.5)  # only j1 touched; j2 should fall back to its current actual; sends immediately

    assert len(fake_ri.arm_goal_calls) == 1
    names, positions, duration, _ = fake_ri.arm_goal_calls[0]
    assert names == ARM_JOINTS
    assert positions == [0.5, 0.2]
    assert duration == 0.2


def test_stream_rejects_and_sends_nothing_if_a_joint_has_no_data_at_all(fake_ri):
    jcm, _ = make_manager(fake_ri, positions={'j1': 0.05})  # j2 missing entirely
    errors = []
    jcm.error_occurred.connect(errors.append)
    jcm.set_target('j1', 0.5)
    assert len(fake_ri.arm_goal_calls) == 0
    assert errors


# -- throttling: leading edge immediate, then coalesced, then idle --

def test_leading_edge_change_sends_immediately(fake_ri):
    jcm, _ = make_manager(fake_ri)
    jcm.set_target('j1', 0.5)
    assert len(fake_ri.arm_goal_calls) == 1


def test_second_change_within_throttle_window_is_coalesced_not_sent_immediately(fake_ri):
    jcm, _ = make_manager(fake_ri)
    jcm.set_target('j1', 0.5)  # leading edge -- sent
    assert len(fake_ri.arm_goal_calls) == 1

    jcm.set_target('j1', 0.6)  # still inside the throttle window
    assert len(fake_ri.arm_goal_calls) == 1  # not sent again yet
    assert jcm.target('j1') == 0.6  # but the target itself did update


def test_timer_tick_sends_latest_value_while_still_dirty(fake_ri):
    jcm, _ = make_manager(fake_ri)
    jcm.set_target('j1', 0.5)
    jcm.set_target('j1', 0.6)  # coalesced, not yet sent

    jcm._on_arm_send_timer_timeout()  # simulate the throttle interval elapsing
    assert len(fake_ri.arm_goal_calls) == 2
    names, positions, _, _ = fake_ri.arm_goal_calls[-1]
    assert dict(zip(names, positions))['j1'] == 0.6


def test_timer_goes_idle_once_nothing_changed(fake_ri):
    jcm, _ = make_manager(fake_ri)
    jcm.set_target('j1', 0.5)
    assert jcm._arm_send_timer.isActive() is True

    jcm._on_arm_send_timer_timeout()  # nothing changed since the leading-edge send
    assert len(fake_ri.arm_goal_calls) == 1  # no extra goal sent "just because"
    assert jcm._arm_send_timer.isActive() is False  # stopped itself, not polling forever


def test_arm_and_gripper_streams_throttle_independently(fake_ri):
    jcm, _ = make_manager(fake_ri)
    jcm.set_target('j1', 0.5)       # leading-edge arm send
    jcm.set_target('gripper', 1.0)  # leading-edge gripper send -- its own timer
    assert len(fake_ri.arm_goal_calls) == 1
    assert len(fake_ri.gripper_goal_calls) == 1


# -- gripper (genuinely independent) --

def test_gripper_target_sends_only_gripper_goal(fake_ri):
    jcm, _ = make_manager(fake_ri)
    jcm.set_target('gripper', 1.0)
    assert len(fake_ri.gripper_goal_calls) == 1
    assert len(fake_ri.arm_goal_calls) == 0
    position, max_effort, _ = fake_ri.gripper_goal_calls[0]
    assert position == 1.0
    assert max_effort == 50.0


def test_gripper_throttle_coalesces_rapid_changes(fake_ri):
    jcm, _ = make_manager(fake_ri)
    jcm.set_target('gripper', 1.0)  # leading edge
    jcm.set_target('gripper', 0.5)  # coalesced
    assert len(fake_ri.gripper_goal_calls) == 1

    jcm._on_gripper_send_timer_timeout()
    assert len(fake_ri.gripper_goal_calls) == 2
    assert fake_ri.gripper_goal_calls[-1][0] == 0.5


def test_gripper_flush_defense_in_depth_rejects_out_of_range_target(fake_ri):
    jcm, _ = make_manager(fake_ri)
    jcm.set_target('gripper', 1.0)  # sends once, valid
    jcm._targets['gripper'] = 99.0  # bypass set_target's own validation
    errors = []
    jcm.error_occurred.connect(errors.append)
    jcm._flush_gripper()
    assert len(fake_ri.gripper_goal_calls) == 1  # still just the first, valid one
    assert errors


# -- HOME reuse --

def test_go_home_uses_configured_home_position(fake_ri):
    jcm, _ = make_manager(fake_ri)
    jcm.go_home()
    assert len(fake_ri.arm_goal_calls) == 1
    names, positions, duration, _ = fake_ri.arm_goal_calls[0]
    assert names == ARM_JOINTS
    assert positions == [HOME_POSITION['j1'], HOME_POSITION['j2']]
    assert duration == 4.0
    # targets are updated to reflect the commanded home position
    assert jcm.target('j1') == HOME_POSITION['j1']
    assert jcm.target('j2') == HOME_POSITION['j2']


# -- Cartesian (X/Y/Z) jog -- needs the real 5-joint chain kinematics.py
# knows about, so this section uses its own manager/limits, separate from
# the 2-fake-joint setup used everywhere else above.

REAL_ARM_JOINTS = ['Shoulder_Rotation', 'Shoulder_Pitch', 'Elbow', 'Wrist_Pitch', 'Wrist_Roll']
REAL_HOME_POSITION = {n: 0.0 for n in REAL_ARM_JOINTS}


def make_real_limits():
    bounds = {
        'Shoulder_Rotation': (-1.96, 1.96),
        'Shoulder_Pitch': (-1.745, 1.745),
        'Elbow': (-1.5, 1.5),
        'Wrist_Pitch': (-1.658, 1.658),
        'Wrist_Roll': (-2.75, 2.75),
        'gripper': (-0.2, 1.6),
    }
    return JointLimits({name: JointLimit(min_position=lo, max_position=hi) for name, (lo, hi) in bounds.items()})


def make_real_manager(fake_ri, positions=None):
    default_positions = {n: 0.2 for n in REAL_ARM_JOINTS}
    default_positions['gripper'] = 0.3
    rs = FakeRobotState(positions or default_positions)
    jcm = JointControlManager(
        fake_ri, rs, make_real_limits(), REAL_ARM_JOINTS, GRIPPER,
        REAL_HOME_POSITION, 4.0, MOTION_CONFIG,
    )
    return jcm, rs


def test_nudge_cartesian_sends_one_coordinated_goal(fake_ri):
    jcm, _ = make_real_manager(fake_ri)
    assert jcm.nudge_cartesian('x', 0.005) is True
    assert len(fake_ri.arm_goal_calls) == 1
    names, positions, _, _ = fake_ri.arm_goal_calls[0]
    assert names == REAL_ARM_JOINTS
    assert len(positions) == len(REAL_ARM_JOINTS)


def test_nudge_cartesian_actually_moves_tool_tip_by_the_requested_delta(fake_ri):
    from so101_dashboard.robot import kinematics
    jcm, rs = make_real_manager(fake_ri)
    seed = [rs.positions[n] for n in REAL_ARM_JOINTS]
    x0, y0, z0 = kinematics.forward(seed)

    jcm.nudge_cartesian('z', 0.01)
    names, positions, _, _ = fake_ri.arm_goal_calls[0]
    x1, y1, z1 = kinematics.forward(positions)
    assert abs(x1 - x0) < 1e-3
    assert abs(y1 - y0) < 1e-3
    assert abs(z1 - z0 - 0.01) < 1e-3


def test_nudge_cartesian_rejects_invalid_axis(fake_ri):
    jcm, _ = make_real_manager(fake_ri)
    errors = []
    jcm.error_occurred.connect(errors.append)
    assert jcm.nudge_cartesian('w', 0.005) is False
    assert len(fake_ri.arm_goal_calls) == 0
    assert errors


def test_nudge_cartesian_fails_cleanly_with_no_joint_data(fake_ri):
    rs = FakeRobotState({})  # nothing known yet
    jcm = JointControlManager(
        fake_ri, rs, make_real_limits(), REAL_ARM_JOINTS, GRIPPER, REAL_HOME_POSITION, 4.0, MOTION_CONFIG,
    )
    errors = []
    jcm.error_occurred.connect(errors.append)
    assert jcm.nudge_cartesian('x', 0.005) is False
    assert len(fake_ri.arm_goal_calls) == 0
    assert errors


def test_nudge_cartesian_composes_off_last_commanded_target_not_stale_actual(fake_ri):
    jcm, rs = make_real_manager(fake_ri)
    jcm.nudge_cartesian('x', 0.005)  # leading-edge send
    # robot hasn't actually moved yet (rs.positions unchanged) -- a second
    # jog must still compose off the first jog's TARGET, not re-seed from
    # the stale actual position, or repeated clicks would cancel out.
    jcm.nudge_cartesian('x', 0.005)  # coalesced -- still inside the throttle window
    jcm._on_arm_send_timer_timeout()  # simulate the throttle interval elapsing, flush it

    from so101_dashboard.robot import kinematics
    seed = [rs.positions[n] for n in REAL_ARM_JOINTS]
    x0, _, _ = kinematics.forward(seed)
    names, positions, _, _ = fake_ri.arm_goal_calls[-1]
    x1, _, _ = kinematics.forward(positions)
    assert abs((x1 - x0) - 0.01) < 1e-3  # two 5mm jogs really added up to 10mm


def test_current_cartesian_position_none_until_all_joints_known(fake_ri):
    rs = FakeRobotState({'Shoulder_Rotation': 0.0})  # missing the rest
    jcm = JointControlManager(
        fake_ri, rs, make_real_limits(), REAL_ARM_JOINTS, GRIPPER, REAL_HOME_POSITION, 4.0, MOTION_CONFIG,
    )
    assert jcm.current_cartesian_position() is None


def test_current_cartesian_position_matches_forward_kinematics(fake_ri):
    from so101_dashboard.robot import kinematics
    jcm, rs = make_real_manager(fake_ri)
    expected = kinematics.forward([rs.positions[n] for n in REAL_ARM_JOINTS])
    got = jcm.current_cartesian_position()
    assert got is not None
    for a, b in zip(got, expected):
        assert abs(a - b) < 1e-9
