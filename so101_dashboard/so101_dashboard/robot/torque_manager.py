"""The ONLY component allowed to call /toggle_torque. Owns the torque
state machine described in the project spec:

    TORQUE_ENABLED --FREE MOVE--> TOGGLING --success--> FREE_MOVE
    FREE_MOVE --HOLD--> CAPTURING_POSITION --fresh, valid state-->
        PREPARING_HOLD --> TOGGLING --success--> HOLDING (arm goal accepted)

Any service failure, action rejection, stale joint state, or invalid
captured position lands in ERROR, never silently assumes torque changed,
and never sends a hold command without a confirmed, validated, fresh
capture. UNKNOWN is the only startup state -- /toggle_torque gives no
state feedback, so the app never guesses.
"""
from __future__ import annotations

import time
import uuid
from enum import Enum, auto
from typing import Dict, List, Optional, Sequence

from PySide6.QtCore import QObject, QTimer, Signal

from .joint_limits import JointLimits
from ..ros.robot_interface import RobotInterface
from ..utils.logging import get_logger
from ..utils.signals import ActionOutcome, JointStateSnapshot, ServiceCallOutcome

_log = get_logger('robot.torque_manager')


class TorqueState(Enum):
    UNKNOWN = auto()             # startup, or after an acknowledged error
    TORQUE_ENABLED = auto()
    TORQUE_DISABLED = auto()     # disabled but not (yet) in an intentional free-move session
    TOGGLING = auto()            # /toggle_torque call in flight -- all buttons disabled
    FREE_MOVE = auto()
    CAPTURING_POSITION = auto()
    PREPARING_HOLD = auto()
    HOLDING = auto()
    ERROR = auto()


# States from which each user action is actually accepted.
_FREE_MOVE_ALLOWED_FROM = {TorqueState.TORQUE_ENABLED, TorqueState.HOLDING, TorqueState.UNKNOWN}
_HOLD_ALLOWED_FROM = {TorqueState.FREE_MOVE, TorqueState.TORQUE_DISABLED}


class TorqueManager(QObject):
    state_changed = Signal(object, str)   # (TorqueState, detail message)
    error_occurred = Signal(str)          # user-facing error message
    gripper_lock_changed = Signal(bool)   # confirmed (not just requested) gripper-lock state

    def __init__(
        self,
        robot_interface: RobotInterface,
        joint_limits: JointLimits,
        arm_joint_names: Sequence[str],
        gripper_joint_name: str,
        motion_config: dict,
        safety_config: dict,
        parent: Optional[QObject] = None,
    ) -> None:
        super().__init__(parent)
        self._ri = robot_interface
        self._joint_limits = joint_limits
        self._arm_joint_names: List[str] = list(arm_joint_names)
        self._gripper_joint_name = gripper_joint_name
        self._all_hold_joint_names: List[str] = self._arm_joint_names + [gripper_joint_name]

        self._hold_duration = float(motion_config.get('hold_trajectory_duration', 1.0))
        self._gripper_max_effort = float(motion_config.get('gripper_hold_max_effort', 50.0))
        self._joint_state_timeout = float(safety_config.get('joint_state_timeout', 0.5))
        self._action_timeout_ms = int(float(safety_config.get('action_goal_timeout', 5.0)) * 1000)

        self._state: TorqueState = TorqueState.UNKNOWN
        self._pending_request_id: Optional[str] = None
        self._expected_after_toggle: Optional[TorqueState] = None
        self._capture_request_time: float = 0.0
        self._captured_positions: Dict[str, float] = {}
        self._hold_arm_request_id: Optional[str] = None

        # Whether the gripper keeps its torque (stays holding whatever it's
        # gripping) during free-move. Matches the hardware interface's own
        # default -- confirmed, not assumed, once the first /set_gripper_lock
        # response comes back; independent of the main TorqueState machine
        # since it's a standing preference, not a one-shot action.
        self._gripper_lock_enabled = True
        self._pending_gripper_lock_request_id: Optional[str] = None
        self._pending_gripper_lock_value: bool = True

        self._capture_timer = QTimer(self)
        self._capture_timer.setSingleShot(True)
        self._capture_timer.timeout.connect(self._on_capture_timeout)

        self._ri.torque_result.connect(self._on_torque_result)
        self._ri.gripper_lock_result.connect(self._on_gripper_lock_result)
        self._ri.trajectory_result.connect(self._on_trajectory_result)
        self._ri.joint_state_received.connect(self._on_joint_state_for_capture)

    # -- read-only state --

    @property
    def state(self) -> TorqueState:
        return self._state

    def can_enter_free_move(self) -> bool:
        return self._state in _FREE_MOVE_ALLOWED_FROM

    def can_hold(self) -> bool:
        return self._state in _HOLD_ALLOWED_FROM

    @property
    def gripper_lock_enabled(self) -> bool:
        """Last CONFIRMED value (not the last requested one, if that
        request hasn't come back yet) -- matches this class's existing
        habit of never assuming an unconfirmed state."""
        return self._gripper_lock_enabled

    # -- public commands: the GUI calls these, never the ROS service directly --

    def set_gripper_lock(self, locked: bool) -> None:
        """Whether the gripper keeps its torque (holds whatever it's
        gripping) the next time FREE MOVE disables torque. Independent of
        the main torque state machine -- can be changed at any time, takes
        effect on the next disable, not retroactively on an arm already in
        free-move."""
        self._pending_gripper_lock_request_id = str(uuid.uuid4())
        self._pending_gripper_lock_value = locked
        self._ri.request_set_gripper_lock(locked, request_id=self._pending_gripper_lock_request_id)

    def enter_free_move(self) -> None:
        if not self.can_enter_free_move():
            _log.warning(f'enter_free_move ignored: state is {self._state.name}')
            return
        _log.info('FREE MOVE requested: cancelling active motion, then disabling torque')
        self._ri.request_cancel_all()
        self._pending_request_id = str(uuid.uuid4())
        self._expected_after_toggle = TorqueState.FREE_MOVE
        self._set_state(TorqueState.TOGGLING, 'disabling torque for free move')
        self._ri.request_toggle_torque(request_id=self._pending_request_id)

    def hold_current_position(self) -> None:
        if not self.can_hold():
            _log.warning(f'hold_current_position ignored: state is {self._state.name}')
            return
        self._captured_positions = {}
        self._capture_request_time = time.monotonic()
        self._set_state(TorqueState.CAPTURING_POSITION, 'waiting for a fresh /joint_states message')
        self._capture_timer.start(int(self._joint_state_timeout * 1000) + 1000)

    def acknowledge_error(self) -> None:
        """User dismisses an ERROR. Returns to UNKNOWN -- a fresh, deliberate
        FREE MOVE press is required next; nothing is assumed."""
        if self._state == TorqueState.ERROR:
            self._set_state(TorqueState.UNKNOWN, 'error acknowledged; torque state must be re-established')

    # -- internal: state transition + logging --

    def _set_state(self, new_state: TorqueState, detail: str = '') -> None:
        _log.info(
            f'Torque state: {self._state.name} -> {new_state.name}'
            + (f' ({detail})' if detail else '')
        )
        self._state = new_state
        self.state_changed.emit(new_state, detail)

    def _fail(self, detail: str, user_message: str) -> None:
        self._capture_timer.stop()
        self._set_state(TorqueState.ERROR, detail)
        self.error_occurred.emit(user_message)

    # -- internal: capture flow (HOLD path) --

    def _on_joint_state_for_capture(self, snapshot: JointStateSnapshot) -> None:
        if self._state != TorqueState.CAPTURING_POSITION:
            return
        candidate = {
            name: snapshot.positions[name]
            for name in self._all_hold_joint_names
            if name in snapshot.positions
        }
        if len(candidate) != len(self._all_hold_joint_names):
            return  # incomplete message, keep waiting for the timer or a full one
        self._captured_positions = candidate
        self._capture_timer.stop()
        self._validate_and_prepare_hold()

    def _on_capture_timeout(self) -> None:
        if self._state != TorqueState.CAPTURING_POSITION:
            return
        self._fail(
            'Timed out waiting for a fresh /joint_states message',
            'Cannot enable torque:\nJoint state is stale or unavailable.',
        )

    def _validate_and_prepare_hold(self) -> None:
        names = list(self._captured_positions.keys())
        positions = list(self._captured_positions.values())
        errors = self._joint_limits.validate_positions(names, positions)
        if errors:
            self._fail(
                '; '.join(errors),
                'Cannot enable torque:\n' + '\n'.join(errors),
            )
            return

        self._set_state(TorqueState.PREPARING_HOLD, 'captured position validated')
        self._pending_request_id = str(uuid.uuid4())
        self._expected_after_toggle = TorqueState.HOLDING
        self._set_state(TorqueState.TOGGLING, 're-enabling torque')
        self._ri.request_toggle_torque(request_id=self._pending_request_id)

    # -- internal: /set_gripper_lock result --

    def _on_gripper_lock_result(self, outcome: ServiceCallOutcome) -> None:
        if outcome.request_id != self._pending_gripper_lock_request_id:
            return  # stray/late result from a previous request -- ignore
        if not outcome.success:
            self.error_occurred.emit(f'Failed to set gripper lock: {outcome.message}')
            return  # local cached state deliberately left unchanged -- not confirmed
        self._gripper_lock_enabled = self._pending_gripper_lock_value
        self.gripper_lock_changed.emit(self._gripper_lock_enabled)

    # -- internal: /toggle_torque result --

    def _on_torque_result(self, outcome: ServiceCallOutcome) -> None:
        if self._state != TorqueState.TOGGLING or outcome.request_id != self._pending_request_id:
            return  # stray/late result from a previous cycle -- ignore, never misapply it

        if not outcome.success:
            self._fail(
                f'Torque service call failed: {outcome.message}',
                f'Torque toggle failed: {outcome.message}\n'
                'Torque state is now UNKNOWN -- verify manually before proceeding.',
            )
            return

        if self._expected_after_toggle == TorqueState.FREE_MOVE:
            self._set_state(TorqueState.FREE_MOVE, outcome.message)
        elif self._expected_after_toggle == TorqueState.HOLDING:
            self._send_hold_commands()
        else:
            self._fail('Torque toggled with no expected target state', 'Internal error: unexpected torque transition.')

    # -- internal: send the captured position back as the hold target --

    def _send_hold_commands(self) -> None:
        arm_positions = [self._captured_positions[n] for n in self._arm_joint_names]
        self._hold_arm_request_id = str(uuid.uuid4())
        _log.info(f'Commanding hold position for {self._arm_joint_names}: {arm_positions}')
        self._ri.request_arm_goal(
            self._arm_joint_names, arm_positions, self._hold_duration,
            request_id=self._hold_arm_request_id,
        )
        if self._gripper_joint_name in self._captured_positions:
            self._ri.request_gripper_goal(
                self._captured_positions[self._gripper_joint_name],
                self._gripper_max_effort,
                request_id=f'{self._hold_arm_request_id}-gripper',
            )
        # HOLDING is only confirmed once the arm goal is actually accepted
        # (see _on_trajectory_result) -- not assumed here.

    def _on_trajectory_result(self, outcome: ActionOutcome) -> None:
        if outcome.request_id == self._hold_arm_request_id:
            if self._state != TorqueState.TOGGLING and self._state != TorqueState.HOLDING:
                return
            if not outcome.accepted:
                self._fail(
                    f'Hold goal rejected: {outcome.message}',
                    f'Torque was re-enabled but the hold command was rejected: {outcome.message}\n'
                    'The arm may move unexpectedly -- check it now.',
                )
                return
            if self._state == TorqueState.TOGGLING:
                self._set_state(TorqueState.HOLDING, 'hold goal accepted')
            if not outcome.succeeded:
                _log.warning(f'Hold goal did not report success: {outcome.message}')
        elif outcome.request_id and outcome.request_id.endswith('-gripper'):
            if not outcome.accepted:
                _log.warning(f'Gripper hold goal rejected: {outcome.message}')
            elif not outcome.succeeded:
                _log.warning(f'Gripper hold goal did not report success: {outcome.message}')
        # else: result for some other in-flight goal (waypoint/trajectory
        # playback etc.) -- not this manager's concern, ignore.
