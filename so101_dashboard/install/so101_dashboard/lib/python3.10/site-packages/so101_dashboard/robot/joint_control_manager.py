"""Manual per-joint control ("teaching pendant"): tracks a target value per
joint, validated against JointLimits, and streams it through the exact
same RobotInterface calls waypoints/programs/HOME already use -- no
separate hardware pathway.

Live-jog interaction model (no explicit Apply step): every time the user
moves a slider, edits the spinbox, or presses a nudge button, that's
immediately sent as a goal -- throttled to at most one goal per
`joint_control_stream_interval_ms` per action client (arm and gripper
throttle independently) so a fast drag doesn't flood the controller with
a goal per pixel. The first change after being idle is sent right away
(no perceptible lag at the start of a drag); while the drag continues,
each throttle-interval tick re-sends the latest value; once nothing has
changed for one interval, sending stops on its own -- nothing is ever
sent "just to keep alive".

Architectural constraint that shapes this whole module: arm_controller
does not have allow_partial_joints_goal set, so every FollowJointTrajectory
goal must specify ALL arm joints, not a subset. There is no way to command
"just one joint" at the controller level -- every arm send is a
coordinated goal for the whole arm, using each joint's current target and
falling back to its current actual position for any joint the user hasn't
touched. The gripper is the one genuinely independent single-joint move,
via its own GripperCommand action. JointTrajectoryController (and
GripperActionController) both accept a new goal while one is still
executing and preempt the old one -- that's what makes this rapid-fire
streaming behave like smooth jogging instead of a queue of stale moves;
each new goal gets its own request_id so a stale goal's late result is
just ignored by the existing request-id-gated result matching, same
pattern as every other manager in this app.

Not gated on torque state (by request): commands are sent regardless of
what TorqueManager reports. If torque is genuinely off, the hardware
interface's own write() already no-ops position commands at the servo
level, so this doesn't bypass any actual safety mechanism.

Cartesian (X/Y/Z) jog: nudge_cartesian() solves inverse kinematics (see
kinematics.py) for a small position step from the arm's current
commanded targets, then applies the resulting joint positions as one
atomic batch (see _set_targets_batch) so the whole arm moves together
through the SAME throttled streaming pathway as manual per-joint jogging
-- not a separate motion pathway. If IK doesn't converge (target outside
the reachable workspace, or a numerical failure), nothing is sent and
error_occurred explains why.
"""
from __future__ import annotations

import uuid
from typing import Dict, List, Optional, Tuple

from PySide6.QtCore import QObject, QTimer, Signal

from . import kinematics
from .joint_limits import JointLimits
from .robot_state import RobotState
from ..ros.robot_interface import RobotInterface
from ..utils.conversions import is_finite
from ..utils.logging import get_logger
from ..utils.signals import ActionOutcome

_log = get_logger('robot.joint_control_manager')


class JointControlManager(QObject):
    targets_changed = Signal()          # any target value changed (or newly initialized)
    error_occurred = Signal(str)
    apply_result = Signal(object)        # ActionOutcome, for the coordinated arm stream
    gripper_apply_result = Signal(object)  # ActionOutcome, for the gripper stream

    def __init__(
        self,
        robot_interface: RobotInterface,
        robot_state: RobotState,
        joint_limits: JointLimits,
        arm_joint_names: List[str],
        gripper_joint_name: str,
        home_position: Dict[str, float],
        home_duration_sec: float,
        motion_config: dict,
        parent: Optional[QObject] = None,
    ) -> None:
        super().__init__(parent)
        self._ri = robot_interface
        self._robot_state = robot_state
        self._joint_limits = joint_limits
        self._arm_joint_names = list(arm_joint_names)
        self._gripper_joint_name = gripper_joint_name
        self._home_position = dict(home_position)
        self._home_duration_sec = home_duration_sec

        self._stream_duration_sec = float(motion_config.get('joint_control_stream_duration', 0.15))
        stream_interval_ms = int(motion_config.get('joint_control_stream_interval_ms', 100))
        self._gripper_max_effort = float(motion_config.get('gripper_hold_max_effort', 50.0))

        self._targets: Dict[str, float] = {}
        # Joints the user has explicitly set a target for (slider/spin/nudge).
        # This set is only ever ADDED to during normal jogging -- it's
        # cleared wholesale by sync_to_current(), not per-joint, since a
        # touched joint's target is only "stale" relative to the robot
        # having moved via something else entirely, not relative to time.
        self._touched: set = set()
        # Joints whose target has been initialized at least once from live
        # position (see _sync_untouched_targets). Deliberately NOT the same
        # as "not touched" -- see that method's docstring for the real-
        # hardware jitter bug this distinction exists to prevent.
        self._seeded: set = set()
        self._apply_request_id = ''
        self._gripper_apply_request_id = ''

        # Leading-edge + throttled-repeat send: the first change after idle
        # goes out immediately; each timer tick after that re-sends only if
        # something actually changed since the last send, and the timer
        # stops itself once nothing has -- see module docstring.
        self._arm_dirty = False
        self._gripper_dirty = False
        self._arm_send_timer = QTimer(self)
        self._arm_send_timer.setInterval(stream_interval_ms)
        self._arm_send_timer.timeout.connect(self._on_arm_send_timer_timeout)
        self._gripper_send_timer = QTimer(self)
        self._gripper_send_timer.setInterval(stream_interval_ms)
        self._gripper_send_timer.timeout.connect(self._on_gripper_send_timer_timeout)

        robot_state.joint_state_updated.connect(self._sync_untouched_targets)
        robot_interface.trajectory_result.connect(self._on_trajectory_result)

    # -- read-only access --

    def target(self, joint_name: str) -> Optional[float]:
        return self._targets.get(joint_name)

    def current(self, joint_name: str) -> Optional[float]:
        return self._robot_state.positions.get(joint_name)

    def sync_to_current(self) -> None:
        """Discard every joint's "touched" lock and snap all targets back
        to the robot's live actual position.

        Bug this fixes: once a joint was ever dragged in Joint Control it
        stayed "touched" for the rest of the app session -- so if the
        robot was later moved by something else entirely (a Program, a
        Waypoint MOVE TO, manual backdrive) and the user came back to jog
        a DIFFERENT joint, the coordinated arm goal would still include
        that old touched joint's long-stale target, visibly snapping it
        back to wherever it was the last time Joint Control touched it.
        Called automatically whenever the Joint Control tab becomes
        visible (see JointControlWidget.showEvent), and also exposed as
        an explicit "SYNC TO CURRENT" button for the rare case the robot
        moves while the tab is already open."""
        self._touched.clear()
        for name in self._arm_joint_names + [self._gripper_joint_name]:
            current = self._robot_state.positions.get(name)
            if current is not None:
                self._targets[name] = current
                self._seeded.add(name)
        self.targets_changed.emit()

    def current_cartesian_position(self) -> Optional[Tuple[float, float, float]]:
        """(x, y, z) meters of the tool tip, from forward kinematics on the
        arm's live actual joint positions -- None until all arm joints
        have reported at least once."""
        positions = [self._robot_state.positions.get(name) for name in self._arm_joint_names]
        if any(p is None for p in positions):
            return None
        return kinematics.forward(positions)

    # -- target editing: validates, stores, and streams a goal --

    def set_target(self, joint_name: str, value: float) -> bool:
        """Validate and store a new target, then immediately (throttled)
        send it. Returns True if accepted; on rejection the previous
        target is left untouched (never silently clamped) and
        error_occurred explains why -- nothing is sent in that case."""
        if not is_finite(value):
            self.error_occurred.emit(f'{joint_name}: target must be a finite number')
            return False
        err = self._joint_limits.validate_position(joint_name, value)
        if err:
            self.error_occurred.emit(f'Rejected: {err}')
            return False
        self._targets[joint_name] = value
        self._touched.add(joint_name)
        self.targets_changed.emit()
        if joint_name == self._gripper_joint_name:
            self._mark_gripper_dirty()
        else:
            self._mark_arm_dirty()
        return True

    def nudge(self, joint_name: str, delta: float) -> bool:
        """Adjust joint_name's target by delta (from a step button), based
        on the current target if set, else the current actual position."""
        base = self._targets.get(joint_name, self._robot_state.positions.get(joint_name))
        if base is None:
            self.error_occurred.emit(f'{joint_name}: no data yet, cannot adjust target')
            return False
        return self.set_target(joint_name, base + delta)

    def nudge_cartesian(self, axis: str, delta_m: float) -> bool:
        """Move the tool tip by delta_m meters along one base-frame axis
        ('x', 'y', or 'z'), solving IK from the arm's current commanded
        targets (so repeated jog clicks compose off the last commanded
        pose, same as per-joint nudge()) and applying the result as one
        atomic batch."""
        if axis not in ('x', 'y', 'z'):
            self.error_occurred.emit(f"Invalid Cartesian axis '{axis}'")
            return False

        seed = []
        for name in self._arm_joint_names:
            value = self._targets.get(name, self._robot_state.positions.get(name))
            if value is None:
                self.error_occurred.emit(f'Cannot jog: no data yet for {name}')
                return False
            seed.append(value)

        x, y, z = kinematics.forward(seed)
        delta = {'x': (delta_m, 0.0, 0.0), 'y': (0.0, delta_m, 0.0), 'z': (0.0, 0.0, delta_m)}[axis]
        target_xyz = (x + delta[0], y + delta[1], z + delta[2])

        result = kinematics.solve_ik(seed, target_xyz, self._joint_limits, self._arm_joint_names)
        if not result.success:
            self.error_occurred.emit(f'Cartesian jog rejected: {result.message}')
            return False

        return self._set_targets_batch(dict(zip(self._arm_joint_names, result.positions)))

    def _set_targets_batch(self, updates: Dict[str, float]) -> bool:
        """Validate and apply several arm-joint targets atomically, then
        send ONE coordinated goal reflecting all of them together.
        set_target() in a loop would flush after the FIRST joint changes
        (leading-edge send) while the rest are still stale, jerking the
        arm through a wrong intermediate pose -- this exists specifically
        so Cartesian jog never does that."""
        names = list(updates.keys())
        values = [updates[n] for n in names]
        errors = self._joint_limits.validate_positions(names, values)
        if errors:
            self.error_occurred.emit('Cannot send:\n' + '\n'.join(errors))
            return False
        for name, value in updates.items():
            self._targets[name] = value
            self._touched.add(name)
        self.targets_changed.emit()
        self._mark_arm_dirty()
        return True

    def _sync_untouched_targets(self) -> None:
        """One-time seeding, NOT continuous syncing: the first time live
        data arrives for a joint the user hasn't touched, its target gets
        initialized to that position (so streaming never hits "no data
        yet" for it). Deliberately does NOT keep re-syncing on every
        subsequent /joint_states message after that.

        Real-hardware bug this fixes: this used to re-sync on every tick
        unconditionally, so while a different joint was being actively
        dragged (a fresh coordinated goal going out every ~100ms), every
        OTHER "held" joint's target was recomputed from the latest sensor
        reading each time too. Real joint-state feedback has latency and
        noise; chasing it into a "hold" target that gets re-sent with an
        aggressive short duration every cycle turned into a live feedback
        loop -- every joint would visibly jitter/twitch continuously
        while ANY single joint was being jogged, even though each
        individual correction was computed "correctly" at the instant it
        was sent. Seeding once and then holding that exact value steady
        (repeating the identical target every cycle asks the controller
        for zero motion, not a tiny noisy correction) eliminates that
        loop entirely. A joint only gets re-seeded by sync_to_current()
        (tab-show, or the SYNC TO CURRENT button) -- for when the robot
        genuinely moved via something else while this tab wasn't driving
        it, which is a one-off correction, not a per-tick one."""
        changed = False
        for name in self._arm_joint_names + [self._gripper_joint_name]:
            if name in self._touched or name in self._seeded:
                continue
            current = self._robot_state.positions.get(name)
            if current is not None:
                self._targets[name] = current
                self._seeded.add(name)
                changed = True
        if changed:
            self.targets_changed.emit()

    # -- throttled streaming send (the only place this module touches the robot, besides HOME) --

    def _mark_arm_dirty(self) -> None:
        self._arm_dirty = True
        if not self._arm_send_timer.isActive():
            self._flush_arm()
            self._arm_send_timer.start()

    def _on_arm_send_timer_timeout(self) -> None:
        if self._arm_dirty:
            self._flush_arm()
        else:
            self._arm_send_timer.stop()  # nothing changed since the last send -- go idle

    def _mark_gripper_dirty(self) -> None:
        self._gripper_dirty = True
        if not self._gripper_send_timer.isActive():
            self._flush_gripper()
            self._gripper_send_timer.start()

    def _on_gripper_send_timer_timeout(self) -> None:
        if self._gripper_dirty:
            self._flush_gripper()
        else:
            self._gripper_send_timer.stop()

    def _flush_arm(self) -> None:
        """Send one coordinated goal covering every arm joint's current
        target (or current actual position, for any joint never touched)."""
        self._arm_dirty = False
        positions = []
        for name in self._arm_joint_names:
            value = self._targets.get(name, self._robot_state.positions.get(name))
            if value is None:
                self.error_occurred.emit(f'Cannot send: no data yet for {name}')
                return
            positions.append(value)

        errors = self._joint_limits.validate_positions(self._arm_joint_names, positions)
        if errors:
            self.error_occurred.emit('Cannot send:\n' + '\n'.join(errors))
            return

        self._apply_request_id = str(uuid.uuid4())
        self._ri.request_arm_goal(
            self._arm_joint_names, positions, self._stream_duration_sec, request_id=self._apply_request_id
        )

    def _flush_gripper(self) -> None:
        self._gripper_dirty = False
        value = self._targets.get(self._gripper_joint_name)
        if value is None:
            self.error_occurred.emit('No gripper target set yet.')
            return
        err = self._joint_limits.validate_position(self._gripper_joint_name, value)
        if err:
            self.error_occurred.emit(f'Rejected: {err}')
            return

        self._gripper_apply_request_id = str(uuid.uuid4())
        self._ri.request_gripper_goal(
            value, self._gripper_max_effort, request_id=self._gripper_apply_request_id
        )

    def go_home(self) -> None:
        """Reuses the same home_position/home_duration_sec config ControlPanel's
        MOVE HOME already uses -- not a second definition of "home". A
        single explicit send, not part of the streaming/throttling above."""
        try:
            positions = [self._home_position[n] for n in self._arm_joint_names]
        except KeyError as exc:
            self.error_occurred.emit(f'Home position is missing joint {exc}')
            return

        errors = self._joint_limits.validate_positions(self._arm_joint_names, positions)
        if errors:
            self.error_occurred.emit('Cannot move home:\n' + '\n'.join(errors))
            return

        for name, pos in zip(self._arm_joint_names, positions):
            self._targets[name] = pos
        self.targets_changed.emit()

        self._apply_request_id = str(uuid.uuid4())
        self._ri.request_arm_goal(
            self._arm_joint_names, positions, self._home_duration_sec, request_id=self._apply_request_id
        )

    def _on_trajectory_result(self, outcome: ActionOutcome) -> None:
        if outcome.request_id == self._apply_request_id:
            if not outcome.accepted:
                self.error_occurred.emit(f'Joint command rejected: {outcome.message}')
            self.apply_result.emit(outcome)
        elif outcome.request_id == self._gripper_apply_request_id:
            if not outcome.accepted:
                self.error_occurred.emit(f'Gripper command rejected: {outcome.message}')
            self.gripper_apply_result.emit(outcome)
