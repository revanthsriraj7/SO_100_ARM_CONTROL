"""ROBOT STATUS panel: ROS2 connection, joint-state freshness, controller
activity, torque state, recording/rosbag state -- all read from
RobotState, nothing computed locally.
"""
from __future__ import annotations

from PySide6.QtCore import Qt
from PySide6.QtWidgets import QFormLayout, QGroupBox, QLabel, QVBoxLayout, QWidget

from ..app.styles import COLOR_ERROR, COLOR_INFO, COLOR_MUTED, COLOR_OK, COLOR_WARN
from ..robot.robot_state import RobotState, RecordingState
from ..robot.torque_manager import TorqueState
from ..utils.signals import ConnectionStatus

_TORQUE_LABELS = {
    TorqueState.UNKNOWN: ('UNKNOWN', COLOR_MUTED),
    TorqueState.TORQUE_ENABLED: ('ENABLED', COLOR_OK),
    TorqueState.TORQUE_DISABLED: ('DISABLED', COLOR_WARN),
    TorqueState.TOGGLING: ('TRANSITIONING...', COLOR_WARN),
    TorqueState.FREE_MOVE: ('FREE MOVE (DISABLED)', COLOR_WARN),
    TorqueState.CAPTURING_POSITION: ('CAPTURING POSITION...', COLOR_WARN),
    TorqueState.PREPARING_HOLD: ('PREPARING HOLD...', COLOR_WARN),
    TorqueState.HOLDING: ('ENABLED (HOLDING)', COLOR_OK),
    TorqueState.ERROR: ('ERROR', COLOR_ERROR),
}


def _dot(text: str, color: str) -> str:
    return f'<span style="color:{color};">●</span> {text}'


class StatusWidget(QGroupBox):
    def __init__(self, robot_state: RobotState, parent: QWidget = None) -> None:
        super().__init__('ROBOT STATUS', parent)
        self._robot_state = robot_state

        form = QFormLayout()
        self._ros_label = QLabel()
        self._joint_states_label = QLabel()
        self._controller_label = QLabel()
        self._torque_label = QLabel()
        self._mode_label = QLabel()
        self._recording_label = QLabel()
        self._rosbag_label = QLabel()

        for text_label, value_label in (
            ('ROS 2', self._ros_label),
            ('Joint States', self._joint_states_label),
            ('Controller', self._controller_label),
            ('Torque', self._torque_label),
            ('Mode', self._mode_label),
            ('Trajectory Recording', self._recording_label),
            ('ROS Bag', self._rosbag_label),
        ):
            value_label.setTextFormat(Qt.RichText)
            form.addRow(text_label + ':', value_label)

        layout = QVBoxLayout(self)
        layout.addLayout(form)

        robot_state.connection_status_changed.connect(self._on_connection_status)
        robot_state.torque_state_changed.connect(self._on_torque_state)
        robot_state.recording_state_changed.connect(self._on_recording_state)

        self._on_connection_status(ConnectionStatus(False, False, True, False))
        self._on_torque_state(robot_state.torque_state, '')
        self._on_recording_state(robot_state.recording)
        self._mode_label.setText(_dot('MOCK MODE' if robot_state.mock_mode else 'NORMAL', COLOR_WARN if robot_state.mock_mode else COLOR_INFO))

    def _on_connection_status(self, status: ConnectionStatus) -> None:
        self._ros_label.setText(
            _dot('CONNECTED', COLOR_OK) if status.ros_connected else _dot('DISCONNECTED', COLOR_ERROR)
        )
        if status.joint_states_stale:
            self._joint_states_label.setText(_dot('STALE', COLOR_ERROR))
        elif status.joint_states_receiving:
            self._joint_states_label.setText(_dot('RECEIVING', COLOR_OK))
        else:
            self._joint_states_label.setText(_dot('NO DATA', COLOR_MUTED))
        self._controller_label.setText(
            _dot('ACTIVE', COLOR_OK) if status.controller_active else _dot('INACTIVE', COLOR_ERROR)
        )

    def _on_torque_state(self, state: TorqueState, _detail: str) -> None:
        text, color = _TORQUE_LABELS.get(state, ('UNKNOWN', COLOR_MUTED))
        self._torque_label.setText(_dot(text, color))

    def _on_recording_state(self, recording: RecordingState) -> None:
        self._recording_label.setText(
            _dot('ON', COLOR_ERROR) if recording.trajectory_recording else _dot('OFF', COLOR_MUTED)
        )
        self._rosbag_label.setText(
            _dot('ON', COLOR_ERROR) if recording.rosbag_recording else _dot('OFF', COLOR_MUTED)
        )
