"""Owns all RollingBuffers (one per signal-kind x joint), ingests from
RobotState updates, and ticks plot widgets to redraw at a configured rate
independent of how fast telemetry actually arrives -- so a 200 Hz
/joint_states publisher doesn't force 200 Hz repaints.
"""
from __future__ import annotations

import time
from typing import Dict, List, Tuple

import numpy as np
from PySide6.QtCore import QObject, QTimer, Signal

from ..robot.robot_state import RobotState

SIGNAL_POSITION = 'position'
SIGNAL_VELOCITY = 'velocity'
SIGNAL_EFFORT = 'effort'
SIGNAL_COMMANDED = 'commanded'


class PlotManager(QObject):
    tick = Signal()  # fired at update_rate; plot widgets redraw from buffers on this

    def __init__(
        self,
        robot_state: RobotState,
        joint_names: List[str],
        update_rate_hz: float,
        history_seconds: float,
        parent=None,
    ) -> None:
        super().__init__(parent)
        self._robot_state = robot_state
        self._joint_names = list(joint_names)
        self._history_seconds = history_seconds
        self._paused = False
        self._t0 = time.monotonic()

        from .data_buffer import RollingBuffer
        self._buffers: Dict[Tuple[str, str], RollingBuffer] = {}
        for signal in (SIGNAL_POSITION, SIGNAL_VELOCITY, SIGNAL_EFFORT, SIGNAL_COMMANDED):
            for joint in self._joint_names:
                self._buffers[(signal, joint)] = RollingBuffer(history_seconds)

        robot_state.joint_state_updated.connect(self._ingest)

        self._redraw_timer = QTimer(self)
        self._redraw_timer.setInterval(max(30, int(1000.0 / max(update_rate_hz, 1.0))))
        self._redraw_timer.timeout.connect(self.tick.emit)
        self._redraw_timer.start()

    # -- ingestion --

    def _ingest(self) -> None:
        if self._paused:
            return
        now = time.monotonic() - self._t0
        rs = self._robot_state
        for joint in self._joint_names:
            if joint in rs.positions:
                self._buffers[(SIGNAL_POSITION, joint)].append(now, rs.positions[joint])
            if joint in rs.velocities:
                self._buffers[(SIGNAL_VELOCITY, joint)].append(now, rs.velocities[joint])
            if joint in rs.efforts:
                self._buffers[(SIGNAL_EFFORT, joint)].append(now, rs.efforts[joint])
            if joint in rs.commanded_positions:
                self._buffers[(SIGNAL_COMMANDED, joint)].append(now, rs.commanded_positions[joint])

    # -- controls --

    def pause(self) -> None:
        self._paused = True

    def resume(self) -> None:
        self._paused = False

    @property
    def is_paused(self) -> bool:
        return self._paused

    def clear(self) -> None:
        for buf in self._buffers.values():
            buf.clear()
        self._t0 = time.monotonic()

    def set_history_seconds(self, seconds: float) -> None:
        self._history_seconds = seconds
        for buf in self._buffers.values():
            buf.set_history_seconds(seconds)

    # -- reads, used by JointPlot widgets on `tick` --

    def get_series(self, signal: str, joint: str) -> Tuple[np.ndarray, np.ndarray]:
        buf = self._buffers.get((signal, joint))
        if buf is None:
            return np.empty(0), np.empty(0)
        return buf.as_arrays()
