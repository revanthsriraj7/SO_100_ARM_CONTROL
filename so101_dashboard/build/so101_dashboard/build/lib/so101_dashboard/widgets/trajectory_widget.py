"""TRAJECTORIES tab: record actual motion, and load/play back saved
trajectories.
"""
from __future__ import annotations

import os

from PySide6.QtWidgets import (
    QComboBox, QFileDialog, QGroupBox, QHBoxLayout, QInputDialog, QLabel,
    QMessageBox, QPushButton, QVBoxLayout, QWidget,
)

from ..recording.trajectory_recorder import TrajectoryRecorder
from ..robot.trajectory_manager import PlaybackState, TrajectoryManager
from ..storage.trajectory_storage import load_trajectory, save_trajectory
from ..utils.signals import ActionOutcome

_RATE_OPTIONS = [10.0, 50.0, 100.0, 200.0]


class TrajectoryWidget(QWidget):
    def __init__(
        self,
        recorder: TrajectoryRecorder,
        playback_manager: TrajectoryManager,
        default_directory: str,
        parent=None,
    ) -> None:
        super().__init__(parent)
        self._recorder = recorder
        self._playback = playback_manager
        self._default_directory = default_directory
        os.makedirs(default_directory, exist_ok=True)

        layout = QHBoxLayout(self)

        # -- recording --
        record_group = QGroupBox('RECORD ACTUAL MOTION')
        record_layout = QVBoxLayout(record_group)
        record_layout.addWidget(QLabel('Sample rate (Hz):'))
        self._rate_combo = QComboBox()
        self._rate_combo.setEditable(True)
        for r in _RATE_OPTIONS:
            self._rate_combo.addItem(str(r))
        self._rate_combo.setCurrentText('100.0')
        record_layout.addWidget(self._rate_combo)

        self._start_btn = QPushButton('START')
        self._start_btn.clicked.connect(self._on_start)
        self._pause_btn = QPushButton('PAUSE')
        self._pause_btn.clicked.connect(self._on_pause_resume)
        self._pause_btn.setEnabled(False)
        self._stop_btn = QPushButton('STOP')
        self._stop_btn.clicked.connect(self._on_stop)
        self._stop_btn.setEnabled(False)
        self._save_recording_btn = QPushButton('SAVE RECORDING')
        self._save_recording_btn.setEnabled(False)
        self._save_recording_btn.clicked.connect(self._on_save_recording)

        self._record_status = QLabel('Idle -- 0 points')
        record_layout.addWidget(self._start_btn)
        record_layout.addWidget(self._pause_btn)
        record_layout.addWidget(self._stop_btn)
        record_layout.addWidget(self._save_recording_btn)
        record_layout.addWidget(self._record_status)
        record_layout.addStretch(1)
        layout.addWidget(record_group)

        # -- playback --
        playback_group = QGroupBox('PLAYBACK')
        playback_layout = QVBoxLayout(playback_group)
        self._info_label = QLabel('No trajectory loaded')
        playback_layout.addWidget(self._info_label)

        load_btn = QPushButton('LOAD')
        load_btn.clicked.connect(self._on_load)
        self._play_btn = QPushButton('PLAY')
        self._play_btn.clicked.connect(playback_manager.play)
        self._pb_pause_btn = QPushButton('PAUSE')
        self._pb_pause_btn.clicked.connect(self._on_playback_pause_resume)
        self._pb_stop_btn = QPushButton('STOP')
        self._pb_stop_btn.clicked.connect(playback_manager.stop)

        playback_layout.addWidget(load_btn)
        playback_layout.addWidget(self._play_btn)
        playback_layout.addWidget(self._pb_pause_btn)
        playback_layout.addWidget(self._pb_stop_btn)
        playback_layout.addStretch(1)
        layout.addWidget(playback_group)

        self._pending_recording = None

        recorder.recording_changed.connect(self._on_recording_changed)
        recorder.point_recorded.connect(self._on_point_recorded)
        playback_manager.playback_state_changed.connect(self._on_playback_state_changed)
        playback_manager.playback_finished.connect(self._on_playback_finished)
        playback_manager.error_occurred.connect(self._on_playback_error)
        playback_manager.trajectory_loaded.connect(self._on_trajectory_loaded)

    # -- recording --

    def _on_start(self) -> None:
        try:
            rate = float(self._rate_combo.currentText())
        except ValueError:
            QMessageBox.warning(self, 'Trajectories', 'Sample rate must be a number.')
            return
        self._recorder.start(rate)

    def _on_pause_resume(self) -> None:
        if self._recorder.is_paused:
            self._recorder.resume()
            self._pause_btn.setText('PAUSE')
        else:
            self._recorder.pause()
            self._pause_btn.setText('RESUME')

    def _on_stop(self) -> None:
        name, ok = QInputDialog.getText(self, 'Stop Recording', 'Trajectory name:')
        if not ok or not name:
            name = 'trajectory'
        self._pending_recording = self._recorder.stop(name)
        self._save_recording_btn.setEnabled(self._pending_recording is not None)

    def _on_recording_changed(self, active: bool) -> None:
        self._start_btn.setEnabled(not active)
        self._pause_btn.setEnabled(active)
        self._stop_btn.setEnabled(active)
        if active:
            self._pause_btn.setText('PAUSE')

    def _on_point_recorded(self, count: int) -> None:
        self._record_status.setText(f'Recording -- {count} points')

    def _on_save_recording(self) -> None:
        if self._pending_recording is None:
            return
        path, _ = QFileDialog.getSaveFileName(
            self, 'Save Trajectory', os.path.join(self._default_directory, f'{self._pending_recording.name}.yaml'),
            'YAML (*.yaml)',
        )
        if path:
            save_trajectory(path, self._pending_recording)
            self._record_status.setText(f'Saved {len(self._pending_recording.points)} points to {os.path.basename(path)}')

    # -- playback --

    def _on_load(self) -> None:
        path, _ = QFileDialog.getOpenFileName(self, 'Load Trajectory', self._default_directory, 'YAML (*.yaml)')
        if not path:
            return
        try:
            trajectory = load_trajectory(path)
        except Exception as exc:  # noqa: BLE001
            QMessageBox.critical(self, 'Trajectories', f'Failed to load trajectory: {exc}')
            return
        self._playback.load(trajectory)

    def _on_trajectory_loaded(self, trajectory) -> None:
        self._info_label.setText(
            f'{trajectory.name}\nDuration: {trajectory.duration:.2f}s\n'
            f'Points: {len(trajectory.points)}\n'
            f'Sample rate: {trajectory.sample_rate_hz or "?"} Hz'
        )

    def _on_playback_pause_resume(self) -> None:
        if self._playback.state == PlaybackState.PLAYING:
            self._playback.pause()
        elif self._playback.state == PlaybackState.PAUSED:
            self._playback.resume()

    def _on_playback_state_changed(self, state: PlaybackState) -> None:
        self._play_btn.setEnabled(state != PlaybackState.PLAYING)
        self._pb_pause_btn.setEnabled(state in (PlaybackState.PLAYING, PlaybackState.PAUSED))
        self._pb_pause_btn.setText('RESUME' if state == PlaybackState.PAUSED else 'PAUSE')
        self._pb_stop_btn.setEnabled(state != PlaybackState.IDLE)

    def _on_playback_finished(self, outcome: ActionOutcome) -> None:
        if outcome.accepted and outcome.succeeded:
            self._info_label.setText(self._info_label.text() + '\n\nPlayback complete.')

    def _on_playback_error(self, message: str) -> None:
        QMessageBox.warning(self, 'Trajectories', message)
