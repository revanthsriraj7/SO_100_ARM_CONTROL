"""Records actual robot motion from /joint_states at a configurable rate,
independent of both the GUI refresh rate and however fast /joint_states
itself publishes: a QTimer samples whatever the latest received state is,
it doesn't record every single incoming message.
"""
from __future__ import annotations

import time
from typing import List, Optional

from PySide6.QtCore import QObject, QTimer, Signal

from ..ros.robot_interface import RobotInterface
from ..storage.trajectory_storage import TrajectoryData, TrajectoryPointData
from ..utils.logging import get_logger
from ..utils.signals import JointStateSnapshot

_log = get_logger('recording.trajectory_recorder')


class TrajectoryRecorder(QObject):
    recording_changed = Signal(bool)
    point_recorded = Signal(int)   # running count

    def __init__(self, robot_interface: RobotInterface, joint_names: List[str], parent=None) -> None:
        super().__init__(parent)
        self._joint_names = list(joint_names)
        self._latest: Optional[JointStateSnapshot] = None
        self._recording = False
        self._paused = False
        self._start_time = 0.0
        self._points: List[TrajectoryPointData] = []
        self._rate_hz = 100.0

        robot_interface.joint_state_received.connect(self._on_joint_state)

        self._timer = QTimer(self)
        self._timer.timeout.connect(self._sample)

    def _on_joint_state(self, snapshot: JointStateSnapshot) -> None:
        self._latest = snapshot

    @property
    def is_recording(self) -> bool:
        return self._recording

    @property
    def is_paused(self) -> bool:
        return self._paused

    @property
    def point_count(self) -> int:
        return len(self._points)

    def start(self, rate_hz: float) -> None:
        if self._recording:
            return
        self._rate_hz = rate_hz
        self._points = []
        self._start_time = time.monotonic()
        self._recording = True
        self._paused = False
        self._timer.setInterval(max(5, int(1000.0 / max(rate_hz, 1.0))))
        self._timer.start()
        _log.info(f'Trajectory recording started at {rate_hz:.1f} Hz')
        self.recording_changed.emit(True)

    def pause(self) -> None:
        if self._recording and not self._paused:
            self._paused = True
            self._timer.stop()
            _log.info('Trajectory recording paused')

    def resume(self) -> None:
        if self._recording and self._paused:
            self._paused = False
            self._timer.start()
            _log.info('Trajectory recording resumed')

    def stop(self, name: str) -> Optional[TrajectoryData]:
        if not self._recording:
            return None
        self._recording = False
        self._paused = False
        self._timer.stop()
        _log.info(f'Trajectory recording stopped: {len(self._points)} points')
        self.recording_changed.emit(False)
        if not self._points:
            return None
        return TrajectoryData(
            name=name,
            joint_names=self._joint_names,
            points=list(self._points),
            sample_rate_hz=self._rate_hz,
        )

    def _sample(self) -> None:
        if self._latest is None:
            return
        missing = [n for n in self._joint_names if n not in self._latest.positions]
        if missing:
            return  # incomplete snapshot, skip this tick rather than record garbage

        t = time.monotonic() - self._start_time
        positions = [self._latest.positions[n] for n in self._joint_names]
        velocities = [self._latest.velocities.get(n, float('nan')) for n in self._joint_names]
        efforts = [self._latest.efforts.get(n, float('nan')) for n in self._joint_names]
        self._points.append(TrajectoryPointData(
            time_from_start=t, positions=positions, velocities=velocities, efforts=efforts,
        ))
        self.point_recorded.emit(len(self._points))
