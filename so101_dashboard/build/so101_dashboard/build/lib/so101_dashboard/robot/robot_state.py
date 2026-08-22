"""Centralized, read-only-from-widgets application state.

Widgets consume RobotState (connect to its signals, read its attributes);
they never create their own ROS subscribers. RobotState itself only
*reads* -- it mirrors data from RobotInterface (telemetry) and
TorqueManager (torque state); it does not initiate any ROS calls itself.
"""
from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional

from PySide6.QtCore import QObject, QTimer, Signal

from .torque_manager import TorqueState
from ..ros.robot_interface import RobotInterface
from ..utils.signals import ConnectionStatus, ControllerStateSnapshot, JointStateSnapshot


@dataclass
class RecordingState:
    trajectory_recording: bool = False
    rosbag_recording: bool = False


class RobotState(QObject):
    joint_state_updated = Signal()
    connection_status_changed = Signal(object)   # ConnectionStatus
    torque_state_changed = Signal(object, str)    # (TorqueState, detail)
    free_move_changed = Signal(bool)
    recording_state_changed = Signal(object)      # RecordingState

    def __init__(
        self,
        robot_interface: RobotInterface,
        joint_names: List[str],
        gripper_joint_name: str,
        joint_state_timeout: float,
        parent: Optional[QObject] = None,
    ) -> None:
        super().__init__(parent)
        self.joint_names: List[str] = list(joint_names) + [gripper_joint_name]
        self.positions: Dict[str, float] = {}
        self.velocities: Dict[str, float] = {}
        self.efforts: Dict[str, float] = {}
        self.commanded_positions: Dict[str, float] = {}
        self.position_errors: Dict[str, float] = {}

        self.torque_state: TorqueState = TorqueState.UNKNOWN
        self.mock_mode: bool = False
        self.recording = RecordingState()

        self._joint_state_timeout = joint_state_timeout
        self._last_joint_state_at: Optional[float] = None
        self._node_ready = False
        self._node_error: Optional[str] = None

        robot_interface.joint_state_received.connect(self._on_joint_state)
        robot_interface.controller_state_received.connect(self._on_controller_state)
        robot_interface.node_ready.connect(self._on_node_ready)
        robot_interface.node_error.connect(self._on_node_error)

        self._stale_timer = QTimer(self)
        self._stale_timer.setInterval(200)
        self._stale_timer.timeout.connect(self._emit_connection_status)
        self._stale_timer.start()

    # -- consumed by TorqueManager's state_changed signal (connected externally
    #    by application.py, since RobotState must not depend on TorqueManager's
    #    constructor to avoid an import cycle -- TorqueManager depends on
    #    RobotInterface only, RobotState mirrors its output) --

    def on_torque_state_changed(self, state: TorqueState, detail: str) -> None:
        self.torque_state = state
        self.torque_state_changed.emit(state, detail)
        is_free_move = state in (TorqueState.FREE_MOVE, TorqueState.CAPTURING_POSITION)
        self.free_move_changed.emit(is_free_move)

    def set_trajectory_recording(self, active: bool) -> None:
        self.recording.trajectory_recording = active
        self.recording_state_changed.emit(self.recording)

    def set_rosbag_recording(self, active: bool) -> None:
        self.recording.rosbag_recording = active
        self.recording_state_changed.emit(self.recording)

    # -- internal slots --

    def _on_joint_state(self, snapshot: JointStateSnapshot) -> None:
        self.positions.update(snapshot.positions)
        self.velocities.update(snapshot.velocities)
        self.efforts.update(snapshot.efforts)
        self._last_joint_state_at = time.monotonic()
        self.joint_state_updated.emit()

    def _on_controller_state(self, snapshot: ControllerStateSnapshot) -> None:
        for name, desired in zip(snapshot.joint_names, snapshot.desired_positions):
            self.commanded_positions[name] = desired
        for name, err in zip(snapshot.joint_names, snapshot.error_positions):
            self.position_errors[name] = err
        self.joint_state_updated.emit()

    def _on_node_ready(self) -> None:
        self._node_ready = True
        self._node_error = None
        self._emit_connection_status()

    def _on_node_error(self, message: str) -> None:
        self._node_ready = False
        self._node_error = message
        self._emit_connection_status()

    def is_joint_state_stale(self) -> bool:
        if self._last_joint_state_at is None:
            return True
        return (time.monotonic() - self._last_joint_state_at) > self._joint_state_timeout

    def is_receiving_joint_states(self) -> bool:
        return self._last_joint_state_at is not None

    def _emit_connection_status(self) -> None:
        self.connection_status_changed.emit(
            ConnectionStatus(
                ros_connected=self._node_ready,
                joint_states_receiving=self.is_receiving_joint_states(),
                joint_states_stale=self.is_joint_state_stale(),
                # Controller-active detection needs /arm_controller/transition_event,
                # not yet subscribed -- report "unknown as True" would be misleading,
                # so this is intentionally tied to joint-state health for now.
                controller_active=self.is_receiving_joint_states() and not self.is_joint_state_stale(),
            )
        )
