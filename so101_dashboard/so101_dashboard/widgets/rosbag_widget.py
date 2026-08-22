"""ROS BAG tab: topic selection, start/stop via rosbag2_py (through
RobotInterface -- never a hand-rolled file format, never a subprocess).

Duration is tracked locally from the wall-clock time of the start
confirmation, not polled from the ROS thread -- keeps this widget simple
and only shows numbers we can actually stand behind.
"""
from __future__ import annotations

import os
import time
from datetime import datetime
from typing import List

from PySide6.QtCore import QTimer
from PySide6.QtWidgets import (
    QCheckBox, QFileDialog, QGroupBox, QHBoxLayout, QLabel, QMessageBox,
    QPushButton, QVBoxLayout, QWidget,
)

from ..ros.robot_interface import RobotInterface
from ..utils.signals import RosbagOutcome


def _fmt_duration(seconds: float) -> str:
    total = int(seconds)
    h, rem = divmod(total, 3600)
    m, s = divmod(rem, 60)
    return f'{h:02d}:{m:02d}:{s:02d}'


class RosbagWidget(QWidget):
    def __init__(
        self,
        robot_interface: RobotInterface,
        available_topics: List[str],
        default_topics: List[str],
        default_directory: str,
        mock_mode: bool,
        parent=None,
    ) -> None:
        super().__init__(parent)
        self._ri = robot_interface
        self._default_directory = default_directory
        os.makedirs(default_directory, exist_ok=True)
        self._recording = False
        self._start_wall_time = 0.0
        self._bag_path = ''

        layout = QVBoxLayout(self)
        if mock_mode:
            warn = QLabel('MOCK MODE: rosbag recording here is simulated -- no bag file is actually written.')
            warn.setStyleSheet('color: #d29922; font-weight: 600;')
            layout.addWidget(warn)

        topics_group = QGroupBox('ROS BAG -- Topics')
        topics_layout = QVBoxLayout(topics_group)
        self._checkboxes = {}
        for topic in available_topics:
            cb = QCheckBox(topic)
            cb.setChecked(topic in default_topics)
            self._checkboxes[topic] = cb
            topics_layout.addWidget(cb)
        layout.addWidget(topics_group)

        controls = QHBoxLayout()
        self._start_btn = QPushButton('START RECORDING')
        self._start_btn.clicked.connect(self._on_start)
        self._stop_btn = QPushButton('STOP RECORDING')
        self._stop_btn.clicked.connect(self._on_stop)
        self._stop_btn.setEnabled(False)
        controls.addWidget(self._start_btn)
        controls.addWidget(self._stop_btn)
        layout.addLayout(controls)

        status_group = QGroupBox('Status')
        status_layout = QVBoxLayout(status_group)
        self._recording_label = QLabel('Recording: ○')
        self._duration_label = QLabel('Duration: 00:00:00')
        self._path_label = QLabel('Bag path: --')
        self._path_label.setWordWrap(True)
        status_layout.addWidget(self._recording_label)
        status_layout.addWidget(self._duration_label)
        status_layout.addWidget(self._path_label)
        layout.addWidget(status_group)
        layout.addStretch(1)

        self._timer = QTimer(self)
        self._timer.setInterval(1000)
        self._timer.timeout.connect(self._update_duration)

        robot_interface.rosbag_result.connect(self._on_rosbag_result)

    def _selected_topics(self) -> List[str]:
        return [t for t, cb in self._checkboxes.items() if cb.isChecked()]

    def _on_start(self) -> None:
        topics = self._selected_topics()
        if not topics:
            QMessageBox.warning(self, 'ROS Bag', 'Select at least one topic to record.')
            return
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self._bag_path = os.path.join(self._default_directory, f'bag_{stamp}')
        self._ri.request_start_rosbag(topics, self._bag_path)
        self._start_btn.setEnabled(False)

    def _on_stop(self) -> None:
        self._ri.request_stop_rosbag()
        self._stop_btn.setEnabled(False)

    def _on_rosbag_result(self, outcome: RosbagOutcome) -> None:
        if not outcome.success:
            QMessageBox.warning(self, 'ROS Bag', outcome.message)
            self._start_btn.setEnabled(not outcome.recording)
            self._stop_btn.setEnabled(outcome.recording)
            return

        self._recording = outcome.recording
        if self._recording:
            self._start_wall_time = time.monotonic()
            self._path_label.setText(f'Bag path: {self._bag_path}')
            self._timer.start()
        else:
            self._timer.stop()

        self._recording_label.setText('Recording: ●' if self._recording else 'Recording: ○')
        self._recording_label.setStyleSheet('color: #f85149;' if self._recording else 'color: #8b949e;')
        self._start_btn.setEnabled(not self._recording)
        self._stop_btn.setEnabled(self._recording)

    def _update_duration(self) -> None:
        elapsed = time.monotonic() - self._start_wall_time
        self._duration_label.setText(f'Duration: {_fmt_duration(elapsed)}')
