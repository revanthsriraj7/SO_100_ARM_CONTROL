"""TELEMETRY tab: joint selection, Position/Velocity/Effort/Command-vs-
Actual plots, and Pause/Resume/Clear/history controls.
"""
from __future__ import annotations

from typing import List

from PySide6.QtWidgets import (
    QCheckBox, QDoubleSpinBox, QGroupBox, QHBoxLayout, QLabel, QPushButton,
    QTabWidget, QVBoxLayout, QWidget,
)

from ..plotting.joint_plot import JointPlot
from ..plotting.plot_manager import (
    SIGNAL_COMMANDED, SIGNAL_EFFORT, SIGNAL_POSITION, SIGNAL_VELOCITY, PlotManager,
)


class TelemetryWidget(QWidget):
    def __init__(self, plot_manager: PlotManager, joint_names: List[str], history_seconds: float, parent=None) -> None:
        super().__init__(parent)
        self._plot_manager = plot_manager
        self._joint_names = list(joint_names)

        layout = QVBoxLayout(self)

        # -- joint selector + transport controls --
        controls = QHBoxLayout()
        self._checkboxes = {}
        joint_group = QGroupBox('Joints')
        joint_layout = QHBoxLayout(joint_group)
        for name in self._joint_names:
            cb = QCheckBox(name)
            cb.setChecked(True)
            cb.stateChanged.connect(self._on_selection_changed)
            self._checkboxes[name] = cb
            joint_layout.addWidget(cb)
        controls.addWidget(joint_group, stretch=1)

        transport_group = QGroupBox('Controls')
        transport_layout = QHBoxLayout(transport_group)
        self._pause_btn = QPushButton('Pause')
        self._pause_btn.setCheckable(True)
        self._pause_btn.toggled.connect(self._on_pause_toggled)
        clear_btn = QPushButton('Clear')
        clear_btn.clicked.connect(plot_manager.clear)
        transport_layout.addWidget(self._pause_btn)
        transport_layout.addWidget(clear_btn)
        transport_layout.addWidget(QLabel('History (s):'))
        self._history_spin = QDoubleSpinBox()
        self._history_spin.setRange(1.0, 600.0)
        self._history_spin.setValue(history_seconds)
        self._history_spin.valueChanged.connect(plot_manager.set_history_seconds)
        transport_layout.addWidget(self._history_spin)
        controls.addWidget(transport_group)

        layout.addLayout(controls)

        # -- plots, one per tab --
        self._tabs = QTabWidget()
        self._position_plot = JointPlot(plot_manager, 'Joint Position vs Time', 'Position (rad)')
        self._velocity_plot = JointPlot(plot_manager, 'Joint Velocity vs Time', 'Velocity (rad/s)')
        self._effort_plot = JointPlot(plot_manager, 'Joint Effort vs Time', 'Effort')
        self._command_plot = JointPlot(plot_manager, 'Commanded vs Actual Position', 'Position (rad)')
        self._tabs.addTab(self._position_plot, 'Position')
        self._tabs.addTab(self._velocity_plot, 'Velocity')
        self._tabs.addTab(self._effort_plot, 'Effort')
        self._tabs.addTab(self._command_plot, 'Command vs Actual')
        layout.addWidget(self._tabs, stretch=1)

        self._on_selection_changed()

    def _selected_joints(self) -> List[str]:
        return [name for name, cb in self._checkboxes.items() if cb.isChecked()]

    def _on_selection_changed(self) -> None:
        joints = self._selected_joints()
        self._position_plot.set_joints(joints, SIGNAL_POSITION)
        self._velocity_plot.set_joints(joints, SIGNAL_VELOCITY)
        self._effort_plot.set_joints(joints, SIGNAL_EFFORT)
        self._command_plot.set_command_vs_actual(joints, SIGNAL_POSITION, SIGNAL_COMMANDED)

    def _on_pause_toggled(self, paused: bool) -> None:
        self._pause_btn.setText('Resume' if paused else 'Pause')
        if paused:
            self._plot_manager.pause()
        else:
            self._plot_manager.resume()
