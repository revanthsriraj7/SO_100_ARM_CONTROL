"""PROGRAMS tab: merged RoboDK-style station view. A single QTreeWidget
shows every known program as a top-level, expandable node ("dropdown")
with its ordered waypoint/jaw-action elements grouped underneath as
children -- multiple programs coexist in the tree at once, click a
program's row to make it active, click its arrow to expand/collapse its
sequence. A side panel holds the reusable waypoint library (record/jog to
a single target, independent of any program) plus per-element speed
controls for whichever waypoint element is currently selected in the
tree.

This replaces the old separate WAYPOINTS + PROGRAMS tabs -- the waypoint
library concept and program-sequencing concept are still two different
things underneath (WaypointManager vs. ProgramManager), but the user-
facing surface is now one screen instead of two disconnected ones.
"""
from __future__ import annotations

import os
from typing import Dict, Optional, Tuple

from PySide6.QtCore import Qt
from PySide6.QtGui import QColor
from PySide6.QtWidgets import (
    QCheckBox, QComboBox, QDoubleSpinBox, QFileDialog, QFormLayout, QGridLayout, QGroupBox,
    QHBoxLayout, QInputDialog, QLabel, QListWidget, QMessageBox, QPushButton, QTreeWidget,
    QTreeWidgetItem, QVBoxLayout, QWidget,
)

from ..robot.program_manager import ElementState, ExecutionState, ProgramManager
from ..robot.waypoint_manager import WaypointManager
from ..storage.program_storage import ElementType
from ..utils.signals import ActionOutcome

_STATE_ICON = {
    ElementState.PENDING: '○',
    ElementState.RUNNING: '▶',
    ElementState.COMPLETED: '✓',
    ElementState.FAILED: '✗',
}
_STATE_COLOR = {
    ElementState.PENDING: '#8b949e',
    ElementState.RUNNING: '#d29922',
    ElementState.COMPLETED: '#3fb950',
    ElementState.FAILED: '#f85149',
}
_PROGRAM_COLOR_ACTIVE = '#58a6ff'
_PROGRAM_COLOR_INACTIVE = '#c9d1d9'

# Tree item UserRole payloads: ('program', name) for a top-level program
# header, ('element', program_name, index) for one of its children.


class ProgramWidget(QWidget):
    def __init__(
        self,
        program_manager: ProgramManager,
        waypoint_manager: WaypointManager,
        program_directory: str,
        waypoint_directory: str,
        parent=None,
    ) -> None:
        super().__init__(parent)
        self._pm = program_manager
        self._wm = waypoint_manager
        self._program_directory = program_directory
        self._waypoint_directory = waypoint_directory
        os.makedirs(program_directory, exist_ok=True)
        os.makedirs(waypoint_directory, exist_ok=True)

        self._headers: Dict[str, QTreeWidgetItem] = {}
        self._restoring_selection = False
        self._updating_speed_from_code = False
        self._updating_wait_from_code = False
        self._updating_loop_from_code = False

        layout = QVBoxLayout(self)

        self._name_label = QLabel()
        self._name_label.setObjectName('bannerLabel')
        layout.addWidget(self._name_label)

        body = QHBoxLayout()
        body.addWidget(self._build_tree_panel(), stretch=2)
        # Elements and Waypoint Library are laid out in grids/forms (not
        # single-column stacks of full-width controls) specifically so
        # their combined height stays well under a real screen's height
        # without needing to scroll -- see _build_elements_panel/
        # _build_library_panel.
        body.addWidget(self._build_elements_panel(), stretch=1)
        body.addWidget(self._build_library_panel(), stretch=1)
        layout.addLayout(body, stretch=1)

        run_row = QHBoxLayout()
        self._loop_forever_check = QCheckBox('Loop forever')
        self._loop_forever_check.setToolTip('Repeat this program until STOP is pressed, instead of running it once.')
        self._loop_forever_check.toggled.connect(self._on_loop_forever_toggled)
        run_row.addWidget(self._loop_forever_check)
        self._loop_progress_label = QLabel('')
        self._loop_progress_label.setStyleSheet('color: #8b949e;')
        run_row.addWidget(self._loop_progress_label)
        run_row.addStretch(1)

        self._run_btn = QPushButton('▶ RUN')
        self._run_btn.setObjectName('holdButton')
        self._run_btn.clicked.connect(program_manager.run)
        self._stop_btn = QPushButton('■ STOP PROGRAM')
        self._stop_btn.setObjectName('stopButton')
        self._stop_btn.clicked.connect(program_manager.stop)
        self._stop_btn.setEnabled(False)
        run_row.addWidget(self._run_btn)
        run_row.addWidget(self._stop_btn)
        layout.addLayout(run_row)

        program_manager.program_changed.connect(self._refresh_tree)
        program_manager.programs_changed.connect(self._refresh_tree)
        program_manager.programs_changed.connect(self._sync_waypoints_from_programs)
        program_manager.element_state_changed.connect(self._on_element_state_changed)
        program_manager.execution_state_changed.connect(self._on_execution_state_changed)
        program_manager.loop_progress_changed.connect(self._on_loop_progress_changed)
        program_manager.error_occurred.connect(self._on_error)
        waypoint_manager.waypoints_changed.connect(self._refresh_library)
        waypoint_manager.error_occurred.connect(self._on_error)
        waypoint_manager.move_finished.connect(self._on_move_finished)

        self._refresh_tree()
        self._refresh_library()
        # Covers whatever was already bulk-loaded into the registry before
        # this widget existed (Application.load_all_from_directory() runs
        # before MainWindow is constructed) -- programs_changed won't have
        # fired yet for that, so do one pass explicitly on top of the
        # signal connection above, which covers every LOAD from here on.
        self._sync_waypoints_from_programs()

    # ================= tree (programs + grouped elements) =================

    def _build_tree_panel(self) -> QWidget:
        group = QGroupBox('Programs')
        layout = QVBoxLayout(group)

        self._tree = QTreeWidget()
        self._tree.setHeaderHidden(True)
        self._tree.currentItemChanged.connect(self._on_tree_current_item_changed)
        layout.addWidget(self._tree)

        program_row = QHBoxLayout()
        new_btn = QPushButton('NEW')
        new_btn.clicked.connect(self._on_new)
        rename_btn = QPushButton('RENAME')
        rename_btn.clicked.connect(self._on_rename_program)
        delete_btn = QPushButton('DELETE')
        delete_btn.clicked.connect(self._on_delete_program)
        program_row.addWidget(new_btn)
        program_row.addWidget(rename_btn)
        program_row.addWidget(delete_btn)
        layout.addLayout(program_row)

        file_row = QHBoxLayout()
        save_btn = QPushButton('SAVE')
        save_btn.clicked.connect(self._on_save)
        save_as_btn = QPushButton('SAVE AS')
        save_as_btn.clicked.connect(self._on_save_as)
        load_btn = QPushButton('LOAD')
        load_btn.clicked.connect(self._on_load)
        file_row.addWidget(save_btn)
        file_row.addWidget(save_as_btn)
        file_row.addWidget(load_btn)
        layout.addLayout(file_row)

        return group

    def _current_selection(self) -> Optional[Tuple]:
        item = self._tree.currentItem()
        if item is None:
            return None
        return item.data(0, Qt.ItemDataRole.UserRole)

    def _selected_index(self) -> int:
        """Index within the ACTIVE program's elements, or -1 if the
        selection isn't an element row belonging to it."""
        data = self._current_selection()
        if data and data[0] == 'element' and data[1] == self._pm.active_name:
            return data[2]
        return -1

    def _insertion_index(self) -> int:
        idx = self._selected_index()
        return idx + 1 if idx >= 0 else len(self._pm.program.elements)

    def _element_label(self, program_name: str, index: int, element, state: Optional[ElementState] = None) -> str:
        if state is None:
            state = self._pm.element_state(element) if program_name == self._pm.active_name else ElementState.PENDING
        icon = _STATE_ICON[state]
        suffix = ''
        if element.element_type == ElementType.WAYPOINT:
            vel, acc = self._pm.resolved_scaling(element)
            suffix = f'   (v{vel:.2f} / a{acc:.2f})'
        return f'{icon}  {index + 1}. {element.display_name}{suffix}'

    def _refresh_tree(self) -> None:
        selection = self._current_selection()
        self._tree.blockSignals(True)
        self._tree.clear()
        self._headers = {}
        for name in self._pm.program_names:
            program = self._pm.get_program(name)
            active = name == self._pm.active_name
            # loop_count is strictly binary (see ProgramManager._normalize_loop_count):
            # 0 = loop forever, anything else = run once -- no "run N times".
            loop_suffix = '  ×∞' if program.loop_count == 0 else ''
            header = QTreeWidgetItem([f'{name}  ({len(program.elements)}){loop_suffix}'])
            header.setData(0, Qt.ItemDataRole.UserRole, ('program', name))
            font = header.font(0)
            font.setBold(True)
            header.setFont(0, font)
            header.setForeground(0, QColor(_PROGRAM_COLOR_ACTIVE if active else _PROGRAM_COLOR_INACTIVE))
            self._tree.addTopLevelItem(header)
            for i, element in enumerate(program.elements):
                child = QTreeWidgetItem([self._element_label(name, i, element)])
                child.setData(0, Qt.ItemDataRole.UserRole, ('element', name, i))
                state = self._pm.element_state(element) if active else ElementState.PENDING
                child.setForeground(0, QColor(_STATE_COLOR[state]))
                header.addChild(child)
            header.setExpanded(True)
            self._headers[name] = header
        self._tree.blockSignals(False)

        self._name_label.setText(f'Active Program: {self._pm.program.name}')
        self._updating_loop_from_code = True
        self._loop_forever_check.setChecked(self._pm.program.loop_count == 0)
        self._updating_loop_from_code = False
        self._reapply_selection(selection)

    def _on_loop_forever_toggled(self, checked: bool) -> None:
        if self._updating_loop_from_code:
            return
        self._pm.set_loop_count(0 if checked else 1)

    def _on_loop_progress_changed(self, current: int, total: int) -> None:
        # total is always 0 (forever) or 1 (once) -- see _normalize_loop_count.
        self._loop_progress_label.setText(f'Loop {current} (∞)' if total == 0 else '')

    def _reapply_selection(self, selection: Optional[Tuple]) -> None:
        item = None
        if selection is not None:
            if selection[0] == 'program':
                item = self._headers.get(selection[1])
            else:
                _, pname, idx = selection
                header = self._headers.get(pname)
                if header is not None and 0 <= idx < header.childCount():
                    item = header.child(idx)
                else:
                    item = header
        if item is None:
            item = self._headers.get(self._pm.active_name)

        self._restoring_selection = True
        try:
            if item is not None:
                self._tree.setCurrentItem(item)
        finally:
            self._restoring_selection = False
        self._update_speed_panel_for_item(self._tree.currentItem())

    def _on_tree_current_item_changed(self, current, _previous) -> None:
        if self._restoring_selection:
            return
        if current is None:
            self._update_speed_panel_for_item(None)
            return
        data = current.data(0, Qt.ItemDataRole.UserRole)
        if data is None:
            return
        if data[0] == 'program':
            self._pm.select_program(data[1])
        else:
            _, program_name, _index = data
            if program_name != self._pm.active_name:
                self._pm.select_program(program_name)
        self._update_speed_panel_for_item(current)

    def _on_element_state_changed(self, index: int, state: ElementState) -> None:
        header = self._headers.get(self._pm.active_name)
        if header is None or not (0 <= index < header.childCount()):
            return
        elements = self._pm.program.elements
        if index >= len(elements):
            return
        child = header.child(index)
        child.setText(0, self._element_label(self._pm.active_name, index, elements[index], state))
        child.setForeground(0, QColor(_STATE_COLOR[state]))

    # ================= elements panel (edits the active program) =================

    def _build_elements_panel(self) -> QWidget:
        group = QGroupBox('Elements')
        layout = QVBoxLayout(group)

        # -- add: a 2-column grid instead of one full-width button per
        # row, so four "add" actions cost two rows of height, not four.
        add_grid = QGridLayout()
        add_grid.setSpacing(4)
        add_wp_btn = QPushButton('+ Waypoint')
        add_wp_btn.setToolTip("Record the robot's current pose as a new waypoint element.")
        add_wp_btn.clicked.connect(lambda: self._pm.add_waypoint(self._insertion_index()))
        add_wait_btn = QPushButton('+ Wait')
        add_wait_btn.setToolTip('Insert a fixed time delay before the next element.')
        add_wait_btn.clicked.connect(lambda: self._pm.add_wait(self._insertion_index()))
        add_open_btn = QPushButton('+ Jaw Open')
        add_open_btn.clicked.connect(lambda: self._pm.add_jaw_open(self._insertion_index()))
        add_close_btn = QPushButton('+ Jaw Close')
        add_close_btn.clicked.connect(lambda: self._pm.add_jaw_close(self._insertion_index()))
        add_grid.addWidget(add_wp_btn, 0, 0)
        add_grid.addWidget(add_wait_btn, 0, 1)
        add_grid.addWidget(add_open_btn, 1, 0)
        add_grid.addWidget(add_close_btn, 1, 1)
        layout.addLayout(add_grid)

        from_lib_row = QHBoxLayout()
        self._library_combo = QComboBox()
        insert_lib_btn = QPushButton('+ From Library')
        insert_lib_btn.clicked.connect(self._on_insert_from_library)
        from_lib_row.addWidget(self._library_combo, stretch=1)
        from_lib_row.addWidget(insert_lib_btn)
        layout.addLayout(from_lib_row)

        # -- edit: same 2-column grid treatment --
        edit_grid = QGridLayout()
        edit_grid.setSpacing(4)
        rename_btn = QPushButton('RENAME')
        rename_btn.clicked.connect(self._on_rename_element)
        duplicate_btn = QPushButton('DUPLICATE')
        duplicate_btn.clicked.connect(self._on_duplicate_element)
        re_record_btn = QPushButton('RE-RECORD')
        re_record_btn.setToolTip("Overwrite this waypoint's angles with the robot's current pose.")
        re_record_btn.clicked.connect(self._on_re_record_element)
        delete_btn = QPushButton('DELETE')
        delete_btn.clicked.connect(self._on_delete_element)
        up_btn = QPushButton('▲ MOVE UP')
        up_btn.clicked.connect(self._on_move_up)
        down_btn = QPushButton('▼ MOVE DOWN')
        down_btn.clicked.connect(self._on_move_down)
        edit_grid.addWidget(rename_btn, 0, 0)
        edit_grid.addWidget(duplicate_btn, 0, 1)
        edit_grid.addWidget(re_record_btn, 1, 0)
        edit_grid.addWidget(delete_btn, 1, 1)
        edit_grid.addWidget(up_btn, 2, 0)
        edit_grid.addWidget(down_btn, 2, 1)
        layout.addLayout(edit_grid)

        self._execute_selected_btn = QPushButton('▶ EXECUTE SELECTED')
        self._execute_selected_btn.setToolTip(
            'Run just this one waypoint move, wait, or jaw action, without running the rest of the program.'
        )
        self._execute_selected_btn.clicked.connect(self._on_execute_selected)
        layout.addWidget(self._execute_selected_btn)

        # -- selected-element properties: waypoint speed OR wait duration,
        # only one ever visible at a time depending on what's selected, so
        # this never costs more than one small panel's worth of height.
        self._speed_group = QGroupBox('Selected Waypoint Speed')
        self._speed_group.setVisible(False)
        speed_form = QFormLayout(self._speed_group)
        self._speed_velocity_spin = QDoubleSpinBox()
        self._speed_velocity_spin.setRange(0.05, 1.0)
        self._speed_velocity_spin.setSingleStep(0.05)
        self._speed_velocity_spin.valueChanged.connect(self._on_speed_spin_changed)
        speed_form.addRow('Velocity:', self._speed_velocity_spin)
        self._speed_acceleration_spin = QDoubleSpinBox()
        self._speed_acceleration_spin.setRange(0.05, 1.0)
        self._speed_acceleration_spin.setSingleStep(0.05)
        self._speed_acceleration_spin.valueChanged.connect(self._on_speed_spin_changed)
        speed_form.addRow('Acceleration:', self._speed_acceleration_spin)
        reset_btn = QPushButton('Reset to Default')
        reset_btn.clicked.connect(self._on_reset_element_speed)
        speed_form.addRow(reset_btn)
        layout.addWidget(self._speed_group)

        self._wait_group = QGroupBox('Selected Wait')
        self._wait_group.setVisible(False)
        wait_form = QFormLayout(self._wait_group)
        self._wait_duration_spin = QDoubleSpinBox()
        self._wait_duration_spin.setRange(0.0, 300.0)
        self._wait_duration_spin.setSingleStep(0.5)
        self._wait_duration_spin.setSuffix(' s')
        self._wait_duration_spin.valueChanged.connect(self._on_wait_duration_spin_changed)
        wait_form.addRow('Duration:', self._wait_duration_spin)
        layout.addWidget(self._wait_group)

        layout.addStretch(1)
        return group

    def _on_insert_from_library(self) -> None:
        idx = self._library_combo.currentIndex()
        if idx < 0 or idx >= len(self._wm.waypoints):
            return
        wp = self._wm.waypoints[idx]
        self._pm.add_existing_waypoint(wp, self._insertion_index())

    def _on_rename_element(self) -> None:
        idx = self._selected_index()
        if not (0 <= idx < len(self._pm.program.elements)):
            return
        element = self._pm.program.elements[idx]
        if element.element_type != ElementType.WAYPOINT:
            QMessageBox.information(self, 'Rename', 'Only waypoint elements can be renamed.')
            return
        current = element.waypoint.name
        name, ok = QInputDialog.getText(self, 'Rename Waypoint', 'New name:', text=current)
        if ok and name:
            self._pm.rename_element(idx, name)

    def _on_re_record_element(self) -> None:
        idx = self._selected_index()
        if idx < 0:
            return
        element = self._pm.program.elements[idx]
        if element.element_type != ElementType.WAYPOINT:
            QMessageBox.information(self, 'Re-record', 'Only waypoint elements can be re-recorded.')
            return
        if self._pm.re_record_waypoint(idx):
            # Push the correction into the Waypoint Library too (by name),
            # not just the program -- re-recording is meant to fix the
            # waypoint everywhere it's used, and the library is often
            # where it originally came from.
            self._wm.update_or_add(self._pm.program.elements[idx].waypoint)

    def _on_duplicate_element(self) -> None:
        idx = self._selected_index()
        if idx >= 0:
            self._pm.duplicate_element(idx)

    def _on_execute_selected(self) -> None:
        idx = self._selected_index()
        if idx < 0:
            QMessageBox.information(self, 'Execute Selected', 'Select an element in the tree first.')
            return
        self._pm.execute_element(idx)

    def _on_delete_element(self) -> None:
        idx = self._selected_index()
        if idx >= 0:
            self._pm.delete_element(idx)

    def _on_move_up(self) -> None:
        idx = self._selected_index()
        if idx >= 0:
            self._pm.move_up(idx)

    def _on_move_down(self) -> None:
        idx = self._selected_index()
        if idx >= 0:
            self._pm.move_down(idx)

    def _update_speed_panel_for_item(self, item) -> None:
        """Shows exactly one of the "Selected Waypoint Speed" / "Selected
        Wait" panels (whichever matches the selected element's type),
        hiding both otherwise -- never both, never neither's worth of
        wasted space when nothing relevant is selected."""
        data = item.data(0, Qt.ItemDataRole.UserRole) if item else None
        element = None
        if data and data[0] == 'element':
            _, program_name, index = data
            program = self._pm.get_program(program_name)
            if program is not None and 0 <= index < len(program.elements):
                element = program.elements[index]

        is_waypoint = element is not None and element.element_type == ElementType.WAYPOINT
        is_wait = element is not None and element.element_type == ElementType.WAIT
        self._speed_group.setVisible(is_waypoint)
        self._wait_group.setVisible(is_wait)

        if is_waypoint:
            vel, acc = self._pm.resolved_scaling(element)
            self._updating_speed_from_code = True
            self._speed_velocity_spin.setValue(vel)
            self._speed_acceleration_spin.setValue(acc)
            self._updating_speed_from_code = False
        elif is_wait:
            self._updating_wait_from_code = True
            self._wait_duration_spin.setValue(element.wait_duration_sec)
            self._updating_wait_from_code = False

    def _on_speed_spin_changed(self, _value: float) -> None:
        if self._updating_speed_from_code:
            return
        idx = self._selected_index()
        if idx < 0:
            return
        self._pm.set_element_scaling(idx, self._speed_velocity_spin.value(), self._speed_acceleration_spin.value())

    def _on_reset_element_speed(self) -> None:
        idx = self._selected_index()
        if idx < 0:
            return
        self._pm.set_element_scaling(idx, None, None)

    def _on_wait_duration_spin_changed(self, value: float) -> None:
        if self._updating_wait_from_code:
            return
        idx = self._selected_index()
        if idx < 0:
            return
        self._pm.set_wait_duration(idx, value)

    # ================= program-level actions =================

    def _on_new(self) -> None:
        name, ok = QInputDialog.getText(self, 'New Program', 'Program name:', text='Untitled')
        if ok and name:
            self._pm.new_program(name)

    def _on_rename_program(self) -> None:
        name, ok = QInputDialog.getText(self, 'Rename Program', 'New name:', text=self._pm.active_name)
        if ok and name:
            self._pm.rename_program(name)

    def _on_save(self) -> None:
        path = os.path.join(self._program_directory, f'{self._pm.program.name}.json')
        self._pm.save(path)

    def _on_save_as(self) -> None:
        path, _ = QFileDialog.getSaveFileName(
            self, 'Save Program As', os.path.join(self._program_directory, f'{self._pm.program.name}.json'),
            'JSON (*.json)',
        )
        if path:
            name = os.path.splitext(os.path.basename(path))[0]
            self._pm.rename_program(name)
            self._pm.save(path)

    def _on_load(self) -> None:
        path, _ = QFileDialog.getOpenFileName(self, 'Load Program', self._program_directory, 'JSON (*.json)')
        if not path:
            return
        try:
            self._pm.load(path)
        except Exception as exc:  # noqa: BLE001
            QMessageBox.critical(self, 'Load Program', f'Failed to load program:\n{exc}')

    def _on_delete_program(self) -> None:
        name = self._pm.active_name
        path = os.path.join(self._program_directory, f'{name}.json')
        has_file = os.path.exists(path)
        question = f"Remove program '{name}' from this list?" + (
            '\n\nIts saved file is NOT deleted -- use LOAD to bring it back anytime.'
            if has_file else '\n\n(Not saved to a file yet -- this discards it for good.)'
        )
        if QMessageBox.question(self, 'Remove Program', question) != QMessageBox.Yes:
            return
        self._pm.delete_program(name)

    def _on_execution_state_changed(self, state: ExecutionState) -> None:
        running = state == ExecutionState.RUNNING
        self._run_btn.setEnabled(not running)
        self._stop_btn.setEnabled(running)
        self._loop_forever_check.setEnabled(not running)
        self._execute_selected_btn.setEnabled(not running)

    def _on_error(self, message: str) -> None:
        QMessageBox.warning(self, 'Programs', message)

    # ================= waypoint library panel =================

    def _build_library_panel(self) -> QWidget:
        group = QGroupBox('Waypoint Library')
        layout = QVBoxLayout(group)

        self._library_list = QListWidget()
        layout.addWidget(self._library_list, stretch=1)

        btn_grid = QGridLayout()
        btn_grid.setSpacing(4)
        record_btn = QPushButton('RECORD')
        record_btn.clicked.connect(self._on_record)
        rename_btn = QPushButton('RENAME')
        rename_btn.clicked.connect(self._on_rename_library)
        duplicate_btn = QPushButton('DUPLICATE')
        duplicate_btn.clicked.connect(self._on_duplicate_library)
        delete_btn = QPushButton('DELETE')
        delete_btn.clicked.connect(self._on_delete_library)
        btn_grid.addWidget(record_btn, 0, 0)
        btn_grid.addWidget(rename_btn, 0, 1)
        btn_grid.addWidget(duplicate_btn, 1, 0)
        btn_grid.addWidget(delete_btn, 1, 1)
        layout.addLayout(btn_grid)

        move_form = QFormLayout()
        self._duration_spin = QDoubleSpinBox()
        self._duration_spin.setRange(0.2, 30.0)
        self._duration_spin.setValue(3.0)
        self._duration_spin.setSuffix(' s')
        move_form.addRow('Duration:', self._duration_spin)
        self._velocity_spin = QDoubleSpinBox()
        self._velocity_spin.setRange(0.05, 1.0)
        self._velocity_spin.setSingleStep(0.05)
        self._velocity_spin.setValue(0.3)
        move_form.addRow('Velocity:', self._velocity_spin)
        self._acceleration_spin = QDoubleSpinBox()
        self._acceleration_spin.setRange(0.05, 1.0)
        self._acceleration_spin.setSingleStep(0.05)
        self._acceleration_spin.setValue(0.3)
        move_form.addRow('Acceleration:', self._acceleration_spin)
        layout.addLayout(move_form)

        move_row = QHBoxLayout()
        move_btn = QPushButton('MOVE TO')
        move_btn.setToolTip('Jog the robot to the selected waypoint, independent of any program.')
        move_btn.clicked.connect(self._on_move_to)
        stop_btn = QPushButton('STOP')
        stop_btn.clicked.connect(self._wm.stop)
        move_row.addWidget(move_btn)
        move_row.addWidget(stop_btn)
        layout.addLayout(move_row)

        file_row_1 = QHBoxLayout()
        save_btn = QPushButton('SAVE')
        save_btn.clicked.connect(self._on_save_library)
        load_btn = QPushButton('LOAD')
        load_btn.clicked.connect(self._on_load_library)
        file_row_1.addWidget(save_btn)
        file_row_1.addWidget(load_btn)
        layout.addLayout(file_row_1)

        layout.addStretch(1)
        return group

    def _selected_library_index(self) -> int:
        return self._library_list.currentRow()

    def _refresh_library(self) -> None:
        self._library_list.clear()
        self._library_combo.clear()
        for wp in self._wm.waypoints:
            self._library_list.addItem(wp.name)
            self._library_combo.addItem(wp.name)

    def _sync_waypoints_from_programs(self) -> None:
        """Every waypoint element in every known program gets a copy in the
        reusable Waypoint Library automatically -- so waypoints from a
        loaded program file show up there without a separate manual
        import step. Skips (doesn't duplicate) any name already present."""
        for name in self._pm.program_names:
            program = self._pm.get_program(name)
            if program is None:
                continue
            for element in program.elements:
                if element.element_type == ElementType.WAYPOINT and element.waypoint is not None:
                    self._wm.add_existing(element.waypoint)

    def _on_record(self) -> None:
        name, ok = QInputDialog.getText(self, 'Record Waypoint', 'Waypoint name:')
        if ok and name:
            self._wm.record(name)

    def _on_rename_library(self) -> None:
        idx = self._selected_library_index()
        if idx < 0:
            return
        current = self._wm.waypoints[idx].name
        name, ok = QInputDialog.getText(self, 'Rename Waypoint', 'New name:', text=current)
        if ok and name:
            self._wm.rename(idx, name)

    def _on_delete_library(self) -> None:
        idx = self._selected_library_index()
        if idx >= 0:
            self._wm.delete(idx)

    def _on_duplicate_library(self) -> None:
        idx = self._selected_library_index()
        if idx >= 0:
            self._wm.duplicate(idx)

    def _on_move_to(self) -> None:
        idx = self._selected_library_index()
        if idx < 0:
            return
        self._wm.move_to(
            idx, self._duration_spin.value(),
            self._velocity_spin.value(), self._acceleration_spin.value(),
        )

    def _on_save_library(self) -> None:
        path, _ = QFileDialog.getSaveFileName(self, 'Save Waypoints', self._waypoint_directory, 'YAML (*.yaml)')
        if path:
            self._wm.save(path)

    def _on_load_library(self) -> None:
        path, _ = QFileDialog.getOpenFileName(self, 'Load Waypoints', self._waypoint_directory, 'YAML (*.yaml)')
        if path:
            self._wm.load(path)

    def _on_move_finished(self, outcome: ActionOutcome) -> None:
        if not outcome.accepted:
            QMessageBox.warning(self, 'Move To Waypoint', f'Goal rejected: {outcome.message}')
