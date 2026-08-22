"""Joint state table: Position, Velocity, Effort, Commanded Position,
Position Error. Redraws at a fixed configured rate (not on every single
/joint_states callback) so a fast-publishing robot doesn't flood the GUI
event loop with repaints.
"""
from __future__ import annotations

import math
from typing import Optional

from PySide6.QtCore import Qt, QTimer
from PySide6.QtWidgets import QGroupBox, QTableWidget, QTableWidgetItem, QVBoxLayout, QWidget

from ..robot.robot_state import RobotState
from ..utils.conversions import rad_to_deg

_COLUMNS = ['Joint', 'Position', 'Velocity', 'Effort', 'Command', 'Error']
_NA = 'N/A'


def _fmt_deg(value: Optional[float]) -> str:
    if value is None or not math.isfinite(value):
        return _NA
    return f'{rad_to_deg(value):+.2f}°'


def _fmt_plain(value: Optional[float]) -> str:
    if value is None or not math.isfinite(value):
        return _NA
    return f'{value:+.3f}'


class JointTable(QGroupBox):
    def __init__(self, robot_state: RobotState, update_rate_hz: float, parent: QWidget = None) -> None:
        super().__init__('JOINT TELEMETRY', parent)
        self._robot_state = robot_state

        self._table = QTableWidget(len(robot_state.joint_names), len(_COLUMNS))
        self._table.setHorizontalHeaderLabels(_COLUMNS)
        self._table.verticalHeader().setVisible(False)
        self._table.setEditTriggers(QTableWidget.NoEditTriggers)
        self._table.setSelectionMode(QTableWidget.NoSelection)
        self._table.horizontalHeader().setStretchLastSection(True)

        for row, name in enumerate(robot_state.joint_names):
            self._table.setItem(row, 0, QTableWidgetItem(name))
            for col in range(1, len(_COLUMNS)):
                item = QTableWidgetItem(_NA)
                item.setTextAlignment(Qt.AlignCenter)
                self._table.setItem(row, col, item)

        layout = QVBoxLayout(self)
        layout.addWidget(self._table)

        interval_ms = max(30, int(1000.0 / max(update_rate_hz, 1.0)))
        self._timer = QTimer(self)
        self._timer.setInterval(interval_ms)
        self._timer.timeout.connect(self._refresh)
        self._timer.start()

    def _refresh(self) -> None:
        rs = self._robot_state
        for row, name in enumerate(rs.joint_names):
            pos = rs.positions.get(name)
            vel = rs.velocities.get(name)
            eff = rs.efforts.get(name)
            cmd = rs.commanded_positions.get(name)
            err = rs.position_errors.get(name)

            self._table.item(row, 1).setText(_fmt_deg(pos))
            self._table.item(row, 2).setText(_fmt_plain(vel))
            self._table.item(row, 3).setText(_fmt_plain(eff))
            self._table.item(row, 4).setText(_fmt_deg(cmd))
            self._table.item(row, 5).setText(_fmt_deg(err))
