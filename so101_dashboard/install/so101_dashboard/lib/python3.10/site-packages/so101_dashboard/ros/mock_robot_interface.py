"""Synthetic stand-in for RobotInterface, same public signals/methods, no
rclpy/ROS graph involved -- lets the whole GUI (including the torque state
machine) be exercised without a physical robot or a running ROS system.

Torque toggling IS simulated (flips an internal flag, replies with a
simulated ServiceCallOutcome after a short delay) so TorqueManager's state
machine is actually exercisable end to end in mock mode -- but this is
never presented as real: MainWindow keeps a persistent "MOCK MODE" banner
up the whole time this is in use, and every log line here is tagged
[MOCK].
"""
from __future__ import annotations

import math
import time
import uuid
from typing import Dict, Optional, Sequence

from PySide6.QtCore import QObject, QTimer, Signal

from ..utils.logging import get_logger
from ..utils.signals import ActionOutcome, JointStateSnapshot, RosbagOutcome, ServiceCallOutcome

_log = get_logger('ros.mock_robot_interface')


class MockRobotInterface(QObject):
    joint_state_received = Signal(object)
    controller_state_received = Signal(object)   # never emitted in mock mode (no controller_state topic to fake yet)
    torque_result = Signal(object)
    gripper_lock_result = Signal(object)
    trajectory_result = Signal(object)
    rosbag_result = Signal(object)
    node_ready = Signal()
    node_error = Signal(str)

    def __init__(self, joint_names: Sequence[str], gripper_joint_name: str, parent=None) -> None:
        super().__init__(parent)
        self._joint_names = list(joint_names)
        self._gripper_joint_name = gripper_joint_name
        self._all_names = self._joint_names + [gripper_joint_name]
        self._positions: Dict[str, float] = {n: 0.0 for n in self._all_names}
        self._torque_enabled = True
        self._gripper_locked = True
        self._t0 = time.monotonic()

        self._timer = QTimer(self)
        self._timer.setInterval(33)  # ~30 Hz synthetic /joint_states
        self._timer.timeout.connect(self._tick)

    # -- lifecycle: mirrors RobotInterface's QThread API so application.py
    #    doesn't need an if/else for the rest of its wiring --

    def start(self) -> None:
        _log.info('[MOCK] starting synthetic joint state generator')
        self._timer.start()
        self.node_ready.emit()

    def stop(self) -> None:
        self._timer.stop()

    def wait(self, *_args, **_kwargs) -> bool:
        return True

    # -- synthetic telemetry --

    def _tick(self) -> None:
        t = time.monotonic() - self._t0
        for i, name in enumerate(self._joint_names):
            # small, distinct, slow sinusoids per joint so the GUI has
            # something visibly alive to plot/tabulate
            self._positions[name] = 0.3 * math.sin(t * (0.2 + 0.05 * i) + i)
        # gripper idles at a fixed open-ish position in mock mode
        self._positions[self._gripper_joint_name] = 0.4

        velocities = {n: 0.0 for n in self._all_names}
        efforts = {n: 0.0 for n in self._all_names}
        self.joint_state_received.emit(
            JointStateSnapshot(
                stamp_sec=time.time(),
                positions=dict(self._positions),
                velocities=velocities,
                efforts=efforts,
            )
        )

    # -- simulated commands: same signatures as RobotInterface --

    def request_toggle_torque(self, request_id: Optional[str] = None) -> None:
        def _respond() -> None:
            self._torque_enabled = not self._torque_enabled
            msg = 'Torque enabled' if self._torque_enabled else 'Torque disabled'
            _log.info(f'[MOCK] {msg}')
            self.torque_result.emit(
                ServiceCallOutcome(success=True, message=f'[MOCK] {msg}', request_id=request_id)
            )
        QTimer.singleShot(150, _respond)

    def request_set_gripper_lock(self, locked: bool, request_id: Optional[str] = None) -> None:
        def _respond() -> None:
            self._gripper_locked = locked
            msg = f"Gripper lock during free-move {'enabled' if locked else 'disabled'}"
            _log.info(f'[MOCK] {msg}')
            self.gripper_lock_result.emit(
                ServiceCallOutcome(success=True, message=f'[MOCK] {msg}', request_id=request_id)
            )
        QTimer.singleShot(150, _respond)

    def request_arm_goal(
        self,
        joint_names: Sequence[str],
        positions: Sequence[float],
        duration_sec: float,
        request_id: Optional[str] = None,
    ) -> None:
        def _respond() -> None:
            for name, pos in zip(joint_names, positions):
                self._positions[name] = pos
            _log.info(f'[MOCK] arm goal accepted+succeeded: {list(joint_names)} -> {list(positions)}')
            self.trajectory_result.emit(
                ActionOutcome(accepted=True, succeeded=True, message='[MOCK] goal succeeded', request_id=request_id)
            )
        QTimer.singleShot(max(50, int(duration_sec * 200)), _respond)

    def request_gripper_goal(
        self, position: float, max_effort: float, request_id: Optional[str] = None
    ) -> None:
        def _respond() -> None:
            self._positions[self._gripper_joint_name] = position
            self.trajectory_result.emit(
                ActionOutcome(accepted=True, succeeded=True, message='[MOCK] gripper goal succeeded', request_id=request_id)
            )
        QTimer.singleShot(150, _respond)

    def request_full_trajectory_goal(self, joint_names, points, request_id: Optional[str] = None) -> None:
        points = list(points)

        def _respond() -> None:
            if points:
                _, positions, _ = points[-1]
                for name, pos in zip(joint_names, positions):
                    self._positions[name] = pos
            _log.info(f'[MOCK] full trajectory goal accepted+succeeded ({len(points)} points)')
            self.trajectory_result.emit(
                ActionOutcome(accepted=True, succeeded=True, message='[MOCK] trajectory succeeded', request_id=request_id)
            )
        total_duration = points[-1][0] if points else 0.5
        QTimer.singleShot(max(100, int(total_duration * 200)), _respond)

    def request_cancel_all(self) -> None:
        _log.info('[MOCK] cancel_all requested (no-op: nothing tracked in mock mode)')

    def request_start_rosbag(self, topics, output_dir: str) -> None:
        _log.warning('[MOCK] rosbag recording is NOT real in mock mode -- no bag is written')
        self._rosbag_recording = True

        def _respond() -> None:
            self.rosbag_result.emit(RosbagOutcome(
                success=True, recording=True,
                message=f'[MOCK] pretend-recording {len(list(topics))} topics (no file written)',
            ))
        QTimer.singleShot(100, _respond)

    def request_stop_rosbag(self) -> None:
        self._rosbag_recording = False

        def _respond() -> None:
            self.rosbag_result.emit(RosbagOutcome(
                success=True, recording=False, message='[MOCK] pretend-recording stopped',
            ))
        QTimer.singleShot(100, _respond)
