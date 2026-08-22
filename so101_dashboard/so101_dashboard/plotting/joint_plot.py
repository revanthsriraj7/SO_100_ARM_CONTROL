"""A single pyqtgraph plot showing one or more (signal, joint) series.
Used for Position/Velocity/Effort (one curve per selected joint) and for
Command-vs-Actual (two curves per selected joint, dashed vs solid).

Redraws only on PlotManager.tick (a fixed, configurable rate) rather than
on every data point, so plotting never becomes the bottleneck on a
fast-publishing robot.
"""
from __future__ import annotations

from typing import Dict, List, Tuple

import pyqtgraph as pg
from PySide6.QtCore import Qt

from .plot_manager import PlotManager

# A small, high-contrast, colorblind-considerate categorical palette --
# distinct enough for up to 6 joints on a dark background.
JOINT_COLORS = ['#58a6ff', '#3fb950', '#f0883e', '#f778ba', '#a371f7', '#79c0ff']


class JointPlot(pg.PlotWidget):
    def __init__(self, plot_manager: PlotManager, title: str, y_label: str, parent=None) -> None:
        super().__init__(parent)
        self.setBackground('#0d1117')
        self.showGrid(x=True, y=True, alpha=0.3)
        self.setTitle(title, color='#e6edf3', size='11pt')
        self.getAxis('left').setLabel(y_label)
        self.getAxis('bottom').setLabel('Time (s)')
        self.addLegend(offset=(10, 10))
        self.setMouseEnabled(x=True, y=True)  # pan
        # zoom is built-in via scroll wheel / right-drag in pyqtgraph

        self._plot_manager = plot_manager
        self._curves: Dict[str, pg.PlotDataItem] = {}
        # active_specs: list of (curve_key, signal, joint, pen_kwargs, label)
        self._active_specs: List[Tuple[str, str, str, dict, str]] = []

        plot_manager.tick.connect(self._redraw)

    def set_joints(self, joints: List[str], signal: str) -> None:
        """Simple case: one curve per joint, same signal, solid lines."""
        specs = [
            (f'{signal}:{joint}', signal, joint, {'pen': pg.mkPen(JOINT_COLORS[i % len(JOINT_COLORS)], width=2)}, joint)
            for i, joint in enumerate(joints)
        ]
        self._apply_specs(specs)

    def set_command_vs_actual(self, joints: List[str], actual_signal: str, commanded_signal: str) -> None:
        specs = []
        for i, joint in enumerate(joints):
            color = JOINT_COLORS[i % len(JOINT_COLORS)]
            specs.append((
                f'{actual_signal}:{joint}', actual_signal, joint,
                {'pen': pg.mkPen(color, width=2)}, f'{joint} (actual)',
            ))
            specs.append((
                f'{commanded_signal}:{joint}', commanded_signal, joint,
                {'pen': pg.mkPen(color, width=1, style=Qt.DashLine)}, f'{joint} (cmd)',
            ))
        self._apply_specs(specs)

    def _apply_specs(self, specs: List[Tuple[str, str, str, dict, str]]) -> None:
        wanted_keys = {s[0] for s in specs}
        for key in list(self._curves.keys()):
            if key not in wanted_keys:
                self.removeItem(self._curves.pop(key))

        for key, signal, joint, pen_kwargs, label in specs:
            if key not in self._curves:
                self._curves[key] = self.plot([], [], name=label, **pen_kwargs)

        self._active_specs = specs

    def _redraw(self) -> None:
        if not self.isVisible():
            return
        for key, signal, joint, _pen, _label in self._active_specs:
            x, y = self._plot_manager.get_series(signal, joint)
            self._curves[key].setData(x, y)
