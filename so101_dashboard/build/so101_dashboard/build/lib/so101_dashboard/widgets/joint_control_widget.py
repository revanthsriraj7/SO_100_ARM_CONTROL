"""JOINT CONTROL tab: a manual per-joint "teaching pendant" -- slider +
numeric spinbox + step buttons, with a live current-position readout per
joint, sending through JointControlManager only (never touches
RobotInterface directly). Not gated on torque state, by request. No
separate "target" or "error" readout -- the spinbox/slider value IS what
gets sent, there's nothing else to compare it against.

No Apply step: moving a slider, editing the spinbox, or pressing a step
button sends immediately (JointControlManager throttles it internally).
This widget only ever reads back the live current position and forwards
raw input events -- it never decides when to send.

Also includes a Cartesian (X/Y/Z) jog panel -- +/- buttons per axis that
move the tool tip in a straight line in the base frame, via
JointControlManager.nudge_cartesian() (inverse kinematics under the
hood, see robot/kinematics.py), through the exact same streaming
pathway as per-joint jogging.
"""
from __future__ import annotations

from typing import Callable, Dict, Optional

from PySide6.QtCore import Qt, QTimer
from PySide6.QtWidgets import (
    QComboBox, QDoubleSpinBox, QGroupBox, QHBoxLayout, QLabel, QMessageBox,
    QPushButton, QScrollArea, QSlider, QVBoxLayout, QWidget,
)

from ..robot.joint_control_manager import JointControlManager
from ..robot.joint_limits import JointLimits
from ..utils.conversions import deg_to_rad, rad_to_deg

_DEFAULT_LIMIT_DEG = 180.0  # fallback slider range if a joint has no configured limit


class _JointRow(QGroupBox):
    def __init__(
        self,
        manager: JointControlManager,
        joint_name: str,
        display_label: str,
        min_position: Optional[float],
        max_position: Optional[float],
        step_provider: Callable[[], float],
        parent=None,
    ) -> None:
        super().__init__(display_label)
        self._manager = manager
        self._joint_name = joint_name
        self._step_provider = step_provider
        self._min_deg = rad_to_deg(min_position) if min_position is not None else -_DEFAULT_LIMIT_DEG
        self._max_deg = rad_to_deg(max_position) if max_position is not None else _DEFAULT_LIMIT_DEG
        self._updating_from_code = False

        layout = QVBoxLayout(self)

        info_row = QHBoxLayout()
        self._current_label = QLabel('Current: --')
        info_row.addWidget(self._current_label)
        self._target_spin = QDoubleSpinBox()
        self._target_spin.setDecimals(2)
        self._target_spin.setRange(self._min_deg, self._max_deg)
        self._target_spin.setSuffix('°')
        self._target_spin.valueChanged.connect(self._on_spin_changed)
        info_row.addWidget(self._target_spin)
        info_row.addStretch(1)
        layout.addLayout(info_row)

        slider_row = QHBoxLayout()
        minus_btn = QPushButton('-')
        minus_btn.setFixedWidth(28)
        minus_btn.clicked.connect(lambda: self._nudge(-1))
        self._slider = QSlider(Qt.Orientation.Horizontal)
        self._slider.setRange(int(self._min_deg * 100), int(self._max_deg * 100))
        self._slider.valueChanged.connect(self._on_slider_changed)
        plus_btn = QPushButton('+')
        plus_btn.setFixedWidth(28)
        plus_btn.clicked.connect(lambda: self._nudge(1))

        slider_row.addWidget(QLabel(f'{self._min_deg:.0f}°'))
        slider_row.addWidget(minus_btn)
        slider_row.addWidget(self._slider, stretch=1)
        slider_row.addWidget(plus_btn)
        slider_row.addWidget(QLabel(f'{self._max_deg:.0f}°'))
        layout.addLayout(slider_row)

        manager.targets_changed.connect(self._refresh_target)
        self._refresh_target()

    def _nudge(self, sign: int) -> None:
        self._manager.nudge(self._joint_name, deg_to_rad(sign * self._step_provider()))

    def _on_spin_changed(self, deg_value: float) -> None:
        if self._updating_from_code:
            return
        self._manager.set_target(self._joint_name, deg_to_rad(deg_value))

    def _on_slider_changed(self, centideg_value: int) -> None:
        if self._updating_from_code:
            return
        self._manager.set_target(self._joint_name, deg_to_rad(centideg_value / 100.0))

    def _refresh_target(self) -> None:
        target = self._manager.target(self._joint_name)
        if target is None:
            return
        deg = rad_to_deg(target)
        self._updating_from_code = True
        self._target_spin.setValue(deg)
        self._slider.setValue(int(deg * 100))
        self._updating_from_code = False

    def refresh_current(self) -> None:
        current = self._manager.current(self._joint_name)
        if current is not None:
            self._current_label.setText(f'Current: {rad_to_deg(current):+.2f}°')


class _CartesianJogPanel(QGroupBox):
    def __init__(
        self,
        manager: JointControlManager,
        step_provider_m: Callable[[], float],
        parent=None,
    ) -> None:
        super().__init__('Cartesian Jog (base frame)')
        self._manager = manager
        self._step_provider_m = step_provider_m

        layout = QVBoxLayout(self)
        self._position_label = QLabel('Position: --')
        layout.addWidget(self._position_label)

        for axis, label in (('x', 'X'), ('y', 'Y'), ('z', 'Z')):
            row = QHBoxLayout()
            row.addWidget(QLabel(f'{label}:'))
            minus_btn = QPushButton('-')
            minus_btn.setFixedWidth(32)
            minus_btn.clicked.connect(lambda _checked=False, a=axis: self._jog(a, -1))
            plus_btn = QPushButton('+')
            plus_btn.setFixedWidth(32)
            plus_btn.clicked.connect(lambda _checked=False, a=axis: self._jog(a, 1))
            row.addWidget(minus_btn)
            row.addWidget(plus_btn)
            row.addStretch(1)
            layout.addLayout(row)

        layout.addStretch(1)

    def _jog(self, axis: str, sign: int) -> None:
        self._manager.nudge_cartesian(axis, sign * self._step_provider_m())

    def refresh_current(self) -> None:
        pos = self._manager.current_cartesian_position()
        if pos is not None:
            x, y, z = pos
            self._position_label.setText(f'Position: X={x * 1000:+.1f}  Y={y * 1000:+.1f}  Z={z * 1000:+.1f} mm')


class JointControlWidget(QWidget):
    def __init__(
        self,
        manager: JointControlManager,
        joint_limits: JointLimits,
        arm_joint_names: list,
        gripper_joint_name: str,
        telemetry_update_rate: float,
        parent=None,
    ) -> None:
        super().__init__(parent)
        self._manager = manager

        layout = QVBoxLayout(self)

        top_row = QHBoxLayout()
        top_row.addWidget(QLabel('Step:'))
        self._step_combo = QComboBox()
        for label, value in (('0.1°', 0.1), ('0.5°', 0.5), ('1°', 1.0), ('5°', 5.0)):
            self._step_combo.addItem(label, value)
        self._step_combo.setCurrentIndex(2)  # default 1 deg
        top_row.addWidget(self._step_combo)

        top_row.addWidget(QLabel('Cartesian step:'))
        self._cartesian_step_combo = QComboBox()
        for label, value_m in (('1 mm', 0.001), ('5 mm', 0.005), ('10 mm', 0.010), ('20 mm', 0.020)):
            self._cartesian_step_combo.addItem(label, value_m)
        self._cartesian_step_combo.setCurrentIndex(1)  # default 5mm
        top_row.addWidget(self._cartesian_step_combo)
        top_row.addStretch(1)

        sync_btn = QPushButton('SYNC TO CURRENT')
        sync_btn.setToolTip(
            "Refresh every joint's target to the robot's actual live position. Use this if the "
            'robot was moved by something else (a Program, a Waypoint, manual backdrive) while this '
            "tab wasn't open -- normally happens automatically when you switch to this tab."
        )
        sync_btn.clicked.connect(manager.sync_to_current)
        top_row.addWidget(sync_btn)

        home_btn = QPushButton('HOME')
        home_btn.clicked.connect(manager.go_home)
        top_row.addWidget(home_btn)
        layout.addLayout(top_row)

        note = QLabel(
            'Live jog -- no Apply step: moving a slider, typing a target, or pressing +/- sends '
            'immediately. This arm\'s controller requires every move to specify all joints at once, '
            'so moving one row sends a coordinated move for the whole arm using every row\'s current '
            'target (untouched rows just hold their live position). The gripper moves independently.'
        )
        note.setWordWrap(True)
        note.setStyleSheet('color: #8b949e; font-size: 11px;')
        layout.addWidget(note)

        body = QHBoxLayout()

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        rows_container = QWidget()
        rows_layout = QVBoxLayout(rows_container)

        self._rows: Dict[str, _JointRow] = {}
        for i, name in enumerate(arm_joint_names):
            limit = joint_limits.get(name)
            row = _JointRow(
                manager, name, f'J{i + 1} — {name}',
                limit.min_position if limit else None,
                limit.max_position if limit else None,
                step_provider=lambda: self._step_combo.currentData(),
            )
            rows_layout.addWidget(row)
            self._rows[name] = row

        gripper_limit = joint_limits.get(gripper_joint_name)
        gripper_row = _JointRow(
            manager, gripper_joint_name, 'Gripper',
            gripper_limit.min_position if gripper_limit else None,
            gripper_limit.max_position if gripper_limit else None,
            step_provider=lambda: self._step_combo.currentData(),
        )
        rows_layout.addWidget(gripper_row)
        self._rows[gripper_joint_name] = gripper_row

        rows_layout.addStretch(1)
        scroll.setWidget(rows_container)
        body.addWidget(scroll, stretch=2)

        self._cartesian_panel = _CartesianJogPanel(
            manager, step_provider_m=lambda: self._cartesian_step_combo.currentData(),
        )
        body.addWidget(self._cartesian_panel, stretch=1)

        layout.addLayout(body, stretch=1)

        manager.error_occurred.connect(self._on_error)

        self._refresh_timer = QTimer(self)
        self._refresh_timer.setInterval(max(30, int(1000.0 / max(telemetry_update_rate, 1.0))))
        self._refresh_timer.timeout.connect(self._refresh_current)
        self._refresh_timer.start()

    def _refresh_current(self) -> None:
        for row in self._rows.values():
            row.refresh_current()
        self._cartesian_panel.refresh_current()

    def showEvent(self, event) -> None:
        """Fires whenever this tab becomes the visible one (Qt shows a
        QTabWidget page on switch-to, same as any other widget becoming
        visible) -- re-syncs every target to the robot's current actual
        position so jogging never picks up a stale target left over from
        the last time this tab was open, if the robot moved elsewhere
        (a Program, a Waypoint, manual backdrive) in the meantime."""
        super().showEvent(event)
        self._manager.sync_to_current()

    def _on_error(self, message: str) -> None:
        QMessageBox.warning(self, 'Joint Control', message)
