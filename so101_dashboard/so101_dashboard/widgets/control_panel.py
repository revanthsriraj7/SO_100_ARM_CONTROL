"""ROBOT CONTROL panel: FREE MOVE / HOLD CURRENT POSITION / STOP MOTION /
MOVE HOME. This widget only calls TorqueManager (never the ROS service
directly) for anything torque-related, and shows an unmissable banner for
the current torque state per the spec's Section 29 mockups.
"""
from __future__ import annotations

import uuid
from typing import Dict, List, Sequence

from PySide6.QtCore import Qt
from PySide6.QtWidgets import (
    QCheckBox, QGroupBox, QLabel, QMessageBox, QPushButton, QVBoxLayout, QWidget,
)

from ..robot.joint_limits import JointLimits
from ..robot.torque_manager import TorqueManager, TorqueState
from ..ros.robot_interface import RobotInterface
from ..utils.logging import get_logger
from ..utils.signals import ActionOutcome

_log = get_logger('widgets.control_panel')

_BANNER_TEXT = {
    TorqueState.UNKNOWN: ('● TORQUE STATE UNKNOWN', '#8b949e'),
    TorqueState.TORQUE_ENABLED: ('● TORQUE ENABLED', '#3fb950'),
    TorqueState.TORQUE_DISABLED: ('● TORQUE DISABLED', '#d29922'),
    TorqueState.TOGGLING: ('TORQUE TRANSITION...', '#d29922'),
    TorqueState.FREE_MOVE: ('● FREE MOVE ACTIVE / TORQUE DISABLED', '#d29922'),
    TorqueState.CAPTURING_POSITION: ('CAPTURING POSITION...', '#d29922'),
    TorqueState.PREPARING_HOLD: ('PREPARING HOLD...', '#d29922'),
    TorqueState.HOLDING: ('● TORQUE ENABLED / HOLDING', '#3fb950'),
    TorqueState.ERROR: ('● TORQUE ERROR -- SEE DIALOG', '#f85149'),
}


class ControlPanel(QGroupBox):
    def __init__(
        self,
        torque_manager: TorqueManager,
        robot_interface: RobotInterface,
        joint_limits: JointLimits,
        arm_joint_names: Sequence[str],
        home_position: Dict[str, float],
        home_duration_sec: float,
        parent: QWidget = None,
    ) -> None:
        super().__init__('ROBOT CONTROL', parent)
        self._torque_manager = torque_manager
        self._ri = robot_interface
        self._joint_limits = joint_limits
        self._arm_joint_names: List[str] = list(arm_joint_names)
        self._home_position = dict(home_position)
        self._home_duration = home_duration_sec
        self._home_request_id: str = ''

        self._banner = QLabel()
        self._banner.setObjectName('bannerLabel')
        self._banner.setAlignment(Qt.AlignCenter)

        self._free_move_btn = QPushButton('FREE MOVE')
        self._free_move_btn.setObjectName('freeMoveButton')
        self._free_move_btn.clicked.connect(self._on_free_move_clicked)

        self._updating_gripper_lock_from_code = False
        self._gripper_lock_check = QCheckBox('Lock gripper during free move')
        self._gripper_lock_check.setChecked(True)
        self._gripper_lock_check.setToolTip(
            "Keep the gripper's torque enabled -- still holding whatever it's gripping -- while "
            'FREE MOVE disables the arm joints. Uncheck to let the gripper go limp too.'
        )
        self._gripper_lock_check.toggled.connect(self._on_gripper_lock_toggled)

        self._hold_btn = QPushButton('HOLD CURRENT POSITION')
        self._hold_btn.setObjectName('holdButton')
        self._hold_btn.clicked.connect(self._on_hold_clicked)

        self._stop_btn = QPushButton('STOP MOTION')
        self._stop_btn.setObjectName('stopButton')
        self._stop_btn.clicked.connect(self._on_stop_clicked)

        self._home_btn = QPushButton('MOVE HOME')
        self._home_btn.clicked.connect(self._on_home_clicked)

        layout = QVBoxLayout(self)
        layout.addWidget(self._banner)
        layout.addWidget(self._free_move_btn)
        layout.addWidget(self._gripper_lock_check)
        layout.addWidget(self._hold_btn)
        layout.addSpacing(8)
        layout.addWidget(self._stop_btn)
        layout.addSpacing(8)
        layout.addWidget(self._home_btn)
        layout.addStretch(1)

        torque_manager.state_changed.connect(self._on_torque_state_changed)
        torque_manager.error_occurred.connect(self._on_torque_error)
        torque_manager.gripper_lock_changed.connect(self._on_gripper_lock_changed)
        robot_interface.trajectory_result.connect(self._on_trajectory_result)

        self._refresh_buttons(torque_manager.state)

    # -- torque state / banner --

    def _on_torque_state_changed(self, state: TorqueState, _detail: str) -> None:
        text, color = _BANNER_TEXT.get(state, ('UNKNOWN', '#8b949e'))
        self._banner.setText(text)
        self._banner.setStyleSheet(f'background-color: {color}22; color: {color};')
        self._refresh_buttons(state)

    def _on_torque_error(self, message: str) -> None:
        box = QMessageBox(self)
        box.setIcon(QMessageBox.Critical)
        box.setWindowTitle('Torque Error')
        box.setText(message)
        box.exec()
        self._torque_manager.acknowledge_error()

    def _refresh_buttons(self, state: TorqueState) -> None:
        self._free_move_btn.setEnabled(self._torque_manager.can_enter_free_move())
        self._hold_btn.setEnabled(self._torque_manager.can_hold())
        self._home_btn.setEnabled(state in (TorqueState.TORQUE_ENABLED, TorqueState.HOLDING))
        # STOP is always available while connected -- it only cancels motion,
        # never touches torque, so it's never blocked by the torque state.

    def _on_free_move_clicked(self) -> None:
        self._torque_manager.enter_free_move()

    def _on_gripper_lock_toggled(self, checked: bool) -> None:
        if self._updating_gripper_lock_from_code:
            return
        self._torque_manager.set_gripper_lock(checked)

    def _on_gripper_lock_changed(self, locked: bool) -> None:
        """Reflects the CONFIRMED state back into the checkbox -- in case
        the service call failed, this puts the checkbox back to whatever
        it actually is, rather than trusting the click."""
        self._updating_gripper_lock_from_code = True
        self._gripper_lock_check.setChecked(locked)
        self._updating_gripper_lock_from_code = False

    def _on_hold_clicked(self) -> None:
        self._torque_manager.hold_current_position()

    def _on_stop_clicked(self) -> None:
        _log.info('STOP MOTION pressed')
        self._ri.request_cancel_all()

    def _on_home_clicked(self) -> None:
        positions = [self._home_position[name] for name in self._arm_joint_names]
        errors = self._joint_limits.validate_positions(self._arm_joint_names, positions)
        if errors:
            QMessageBox.critical(self, 'Cannot move home', 'Home position is invalid:\n' + '\n'.join(errors))
            return
        self._home_request_id = str(uuid.uuid4())
        _log.info(f'MOVE HOME requested: {dict(zip(self._arm_joint_names, positions))}')
        self._ri.request_arm_goal(
            self._arm_joint_names, positions, self._home_duration, request_id=self._home_request_id
        )

    def _on_trajectory_result(self, outcome: ActionOutcome) -> None:
        if outcome.request_id != self._home_request_id:
            return
        if not outcome.accepted:
            QMessageBox.warning(self, 'Move Home', f'Home goal rejected: {outcome.message}')
        elif not outcome.succeeded:
            _log.warning(f'Home goal finished without success: {outcome.message}')
