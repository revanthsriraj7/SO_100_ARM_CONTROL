"""Owns a *registry* of Programs (ordered waypoint + jaw-action element
sequences) -- not just one -- so the dashboard can hold multiple programs
in memory at once, RoboDK-station-tree style, switch which one is
"active" for editing/running, and load every previously-saved program up
front. Editing operations, persistence, and the sequential execution
state machine all act on whichever program is currently active.

Execution reuses the exact async request/response pattern already
established by WaypointManager/TrajectoryManager/TorqueManager: send one
async goal via RobotInterface, wait for the shared `trajectory_result`
signal to report a match on a locally-generated request_id, then advance
-- never fires the next element until the current one is confirmed done.

Per-waypoint-element speed: each WAYPOINT element carries its own
optional velocity_scaling/acceleration_scaling (None = "use the app's
configured default"). The executor turns that into an actual move
duration with the exact same trapezoidal min-move-duration machinery
WaypointManager.move_to() already uses for the standalone waypoint
library -- one motion-timing implementation, not two.
"""
from __future__ import annotations

import copy
import os
import uuid
from dataclasses import dataclass
from enum import Enum, auto
from typing import Dict, List, Optional

from PySide6.QtCore import QObject, QTimer, Signal

from .joint_limits import JointLimits
from .robot_state import RobotState
from ..ros.robot_interface import RobotInterface
from ..storage.program_storage import ElementType, Program, ProgramElement, load_program, save_program
from ..storage.waypoint_storage import Waypoint
from ..utils.logging import get_logger
from ..utils.signals import ActionOutcome

_log = get_logger('robot.program_manager')


def _normalize_loop_count(count: int) -> int:
    """0 stays 0 (loop forever); everything else -- including any
    arbitrary N a pre-checkbox version of this app might have saved --
    collapses to 1 (run once). Keeps the binary "Loop forever" invariant
    true everywhere, not just in whatever value the current UI happens to
    let you type in."""
    return 0 if count == 0 else 1


class ElementState(Enum):
    PENDING = auto()
    RUNNING = auto()
    COMPLETED = auto()
    FAILED = auto()


class ExecutionState(Enum):
    IDLE = auto()
    RUNNING = auto()
    STOPPED = auto()
    FINISHED = auto()
    FAILED = auto()


class ProgramManager(QObject):
    program_changed = Signal()               # active program's structure or identity changed
    programs_changed = Signal()              # the SET of known programs changed (new/delete/load/rename)
    element_state_changed = Signal(int, object)   # (index, ElementState)
    execution_state_changed = Signal(object)       # ExecutionState
    loop_progress_changed = Signal(int, int)       # (current_loop, total_loops); total==0 means infinite
    error_occurred = Signal(str)

    def __init__(
        self,
        robot_interface: RobotInterface,
        robot_state: RobotState,
        joint_limits: JointLimits,
        arm_joint_names: List[str],
        gripper_joint_name: str,
        motion_config: dict,
        parent: Optional[QObject] = None,
    ) -> None:
        super().__init__(parent)
        self._ri = robot_interface
        self._robot_state = robot_state
        self._joint_limits = joint_limits
        self._arm_joint_names = list(arm_joint_names)
        self._gripper_joint_name = gripper_joint_name

        self._gripper_open = float(motion_config.get('gripper_open_position', 1.57))
        self._gripper_closed = float(motion_config.get('gripper_closed_position', 0.0))
        self._gripper_max_effort = float(motion_config.get('gripper_hold_max_effort', 50.0))
        self._waypoint_duration = float(motion_config.get('program_waypoint_duration', 3.0))
        self._default_velocity_scaling = float(motion_config.get('default_velocity_scaling', 0.3))
        self._default_acceleration_scaling = float(motion_config.get('default_acceleration_scaling', 0.2))

        # Registry of every program currently known to the dashboard --
        # RoboDK-station-tree style: many programs can be held and switched
        # between at once, not just the one being actively edited/run.
        # Keyed by name; dict preserves insertion order for stable tree display.
        self._programs: Dict[str, Program] = {'Untitled': Program(name='Untitled')}
        self._active_name: str = 'Untitled'
        self._element_states: Dict[str, ElementState] = {}
        self._execution_state = ExecutionState.IDLE
        self._current_index = -1
        self._current_request_id = ''
        self._current_loop = 1
        # True while executing exactly one element on its own (see
        # execute_element()) -- reuses the same RUNNING state machine and
        # result-matching as a full run(), but completing that one element
        # goes back to IDLE instead of advancing into the rest of the
        # sequence.
        self._single_element_run = False

        robot_interface.trajectory_result.connect(self._on_trajectory_result)

    # -- read-only access --

    @property
    def program(self) -> Program:
        """The currently active program -- the one editing/run acts on."""
        return self._programs[self._active_name]

    @property
    def active_name(self) -> str:
        return self._active_name

    @property
    def program_names(self) -> List[str]:
        return list(self._programs.keys())

    def get_program(self, name: str) -> Optional[Program]:
        return self._programs.get(name)

    @property
    def execution_state(self) -> ExecutionState:
        return self._execution_state

    @property
    def current_index(self) -> int:
        return self._current_index

    @property
    def current_loop(self) -> int:
        """1-based index of the loop iteration currently running (or the
        last one that ran, once finished/stopped)."""
        return self._current_loop

    def set_loop_count(self, count: int) -> None:
        """Set the active program's RUN behavior: 0 = loop forever until
        STOP, anything else = run once. Deliberately binary, not an
        arbitrary repeat count -- matches the "Loop forever" checkbox,
        the only way this is exposed in the UI. Persisted with the
        program (see storage/program_storage.py); see also
        _normalize_loop_count(), which enforces the same binary invariant
        on anything LOADED from disk, so a file saved by an older version
        of the app with an arbitrary N can't resurrect "N times" behavior
        or a confusing "Loop 3/3" display."""
        if self.is_running():
            self.error_occurred.emit('Cannot change loop count while the program is running.')
            return
        self.program.loop_count = _normalize_loop_count(count)
        self.program_changed.emit()

    def element_state(self, element: ProgramElement) -> ElementState:
        return self._element_states.get(element.element_id, ElementState.PENDING)

    def is_running(self) -> bool:
        return self._execution_state == ExecutionState.RUNNING

    def resolved_scaling(self, element: ProgramElement) -> tuple:
        """(velocity_scaling, acceleration_scaling) an element will actually
        run with -- its own override if set, else the app-wide default."""
        vel = element.velocity_scaling if element.velocity_scaling is not None else self._default_velocity_scaling
        acc = element.acceleration_scaling if element.acceleration_scaling is not None else self._default_acceleration_scaling
        return vel, acc

    # -- program registry: create / switch / delete --

    def _unique_name(self, base: str) -> str:
        if base not in self._programs:
            return base
        n = 2
        while f'{base} ({n})' in self._programs:
            n += 1
        return f'{base} ({n})'

    def new_program(self, name: str = 'Untitled') -> bool:
        """Create a brand-new program and add it to the registry (does not
        replace any existing one -- multiple programs coexist)."""
        if self.is_running():
            self.error_occurred.emit('Cannot start a new program while one is running.')
            return False
        unique = self._unique_name(name)
        self._programs[unique] = Program(name=unique)
        self._active_name = unique
        self._set_execution_state(ExecutionState.IDLE)
        self.programs_changed.emit()
        self.program_changed.emit()
        return True

    def select_program(self, name: str) -> bool:
        """Switch which registered program is active (for editing/running)."""
        if name == self._active_name:
            return True
        if self.is_running():
            self.error_occurred.emit('Cannot switch programs while one is running.')
            return False
        if name not in self._programs:
            return False
        self._active_name = name
        self.program_changed.emit()
        return True

    def rename_program(self, new_name: str) -> bool:
        """Rename the active program (its registry key and Program.name)."""
        new_name = new_name.strip()
        if not new_name or new_name == self._active_name:
            return False
        if new_name in self._programs:
            self.error_occurred.emit(f"A program named '{new_name}' already exists.")
            return False
        program = self._programs.pop(self._active_name)
        program.name = new_name
        self._programs[new_name] = program
        self._active_name = new_name
        self.programs_changed.emit()
        self.program_changed.emit()
        return True

    def delete_program(self, name: str) -> bool:
        """Remove a program from the in-memory registry (does not touch
        any file on disk -- that's the widget's job, since it owns paths)."""
        if name not in self._programs:
            return False
        if name == self._active_name and self.is_running():
            self.error_occurred.emit('Cannot delete a program while it is running.')
            return False
        del self._programs[name]
        if not self._programs:
            self._programs['Untitled'] = Program(name='Untitled')
        if name == self._active_name:
            self._active_name = next(iter(self._programs))
            self._element_states.clear()
            self._set_execution_state(ExecutionState.IDLE)
        self.programs_changed.emit()
        self.program_changed.emit()
        return True

    def set_name(self, name: str) -> None:
        """Backwards-compatible alias for rename_program."""
        self.rename_program(name)

    def clear(self) -> None:
        if self.is_running():
            self.error_occurred.emit('Cannot clear a program while it is running.')
            return
        self.program.elements.clear()
        self._element_states.clear()
        self.program_changed.emit()

    # -- element editing (acts on the active program) --

    def add_waypoint(
        self,
        index: Optional[int] = None,
        velocity_scaling: Optional[float] = None,
        acceleration_scaling: Optional[float] = None,
    ) -> bool:
        """Record the robot's current position as a new waypoint element."""
        names = self._arm_joint_names + [self._gripper_joint_name]
        missing = [n for n in names if n not in self._robot_state.positions]
        if missing:
            self.error_occurred.emit(f'Cannot record waypoint: no data yet for {missing}')
            return False
        positions = [self._robot_state.positions[n] for n in names]
        n_waypoints = sum(1 for e in self.program.elements if e.element_type == ElementType.WAYPOINT)
        wp = Waypoint.create(f'Waypoint {n_waypoints + 1}', names, positions)
        self._insert(
            ProgramElement(
                element_type=ElementType.WAYPOINT, waypoint=wp,
                velocity_scaling=velocity_scaling, acceleration_scaling=acceleration_scaling,
            ),
            index,
        )
        _log.info(f"Program '{self.program.name}': added waypoint '{wp.name}' at index {index}")
        return True

    def add_existing_waypoint(
        self,
        waypoint: Waypoint,
        index: Optional[int] = None,
        velocity_scaling: Optional[float] = None,
        acceleration_scaling: Optional[float] = None,
    ) -> None:
        """Insert a copy of a waypoint from the reusable waypoint library."""
        self._insert(
            ProgramElement(
                element_type=ElementType.WAYPOINT, waypoint=copy.deepcopy(waypoint),
                velocity_scaling=velocity_scaling, acceleration_scaling=acceleration_scaling,
            ),
            index,
        )

    def add_jaw_open(self, index: Optional[int] = None) -> None:
        self._insert(ProgramElement(element_type=ElementType.JAW_OPEN), index)

    def add_jaw_close(self, index: Optional[int] = None) -> None:
        self._insert(ProgramElement(element_type=ElementType.JAW_CLOSE), index)

    def add_wait(self, index: Optional[int] = None, duration_sec: float = 1.0) -> None:
        """Insert a fixed time delay -- the executor just pauses here for
        duration_sec before advancing, no goal sent to the robot at all."""
        self._insert(ProgramElement(element_type=ElementType.WAIT, wait_duration_sec=max(0.0, duration_sec)), index)

    def set_wait_duration(self, index: int, duration_sec: float) -> None:
        if not (0 <= index < len(self.program.elements)):
            return
        el = self.program.elements[index]
        if el.element_type != ElementType.WAIT:
            return
        el.wait_duration_sec = max(0.0, duration_sec)
        self.program_changed.emit()

    def _insert(self, element: ProgramElement, index: Optional[int]) -> None:
        if self.is_running():
            self.error_occurred.emit('Cannot edit a program while it is running.')
            return
        elements = self.program.elements
        if index is None or not (0 <= index <= len(elements)):
            elements.append(element)
        else:
            elements.insert(index, element)
        self._element_states[element.element_id] = ElementState.PENDING
        self.program_changed.emit()

    def re_record_waypoint(self, index: int) -> bool:
        """Overwrite an existing waypoint element's joint angles with the
        robot's CURRENT position, keeping its name/id and place in the
        sequence -- for correcting a waypoint already in a program without
        deleting and re-adding it (and without losing its speed override
        or any other element's position in the list)."""
        if self.is_running():
            self.error_occurred.emit('Cannot edit a program while it is running.')
            return False
        if not (0 <= index < len(self.program.elements)):
            return False
        el = self.program.elements[index]
        if el.element_type != ElementType.WAYPOINT or el.waypoint is None:
            self.error_occurred.emit('Only waypoint elements can be re-recorded.')
            return False
        names = self._arm_joint_names + [self._gripper_joint_name]
        missing = [n for n in names if n not in self._robot_state.positions]
        if missing:
            self.error_occurred.emit(f'Cannot re-record waypoint: no data yet for {missing}')
            return False
        positions = [self._robot_state.positions[n] for n in names]
        updated = Waypoint.create(el.waypoint.name, names, positions)
        el.waypoint = updated
        _log.info(f"Program '{self.program.name}': re-recorded waypoint '{updated.name}' at index {index}")
        self.program_changed.emit()
        return True

    def duplicate_element(self, index: int) -> bool:
        """Insert a copy of an element right after itself -- a waypoint's
        pose (and its speed override) or a jaw action's type, either way
        without re-recording/re-adding from scratch. Matches
        WaypointManager.duplicate()'s "_copy" naming convention for the
        waypoint case."""
        if self.is_running():
            self.error_occurred.emit('Cannot edit a program while it is running.')
            return False
        if not (0 <= index < len(self.program.elements)):
            return False
        el = self.program.elements[index]
        if el.element_type == ElementType.WAYPOINT and el.waypoint is not None:
            wp_copy = Waypoint.create(f'{el.waypoint.name}_copy', el.waypoint.joint_names, el.waypoint.joint_positions)
            new_el = ProgramElement(
                element_type=ElementType.WAYPOINT, waypoint=wp_copy,
                velocity_scaling=el.velocity_scaling, acceleration_scaling=el.acceleration_scaling,
            )
        elif el.element_type == ElementType.WAIT:
            new_el = ProgramElement(element_type=ElementType.WAIT, wait_duration_sec=el.wait_duration_sec)
        else:
            new_el = ProgramElement(element_type=el.element_type)
        self.program.elements.insert(index + 1, new_el)
        self._element_states[new_el.element_id] = ElementState.PENDING
        _log.info(f"Program '{self.program.name}': duplicated element {index} ('{el.display_name}')")
        self.program_changed.emit()
        return True

    def rename_element(self, index: int, new_name: str) -> None:
        if not (0 <= index < len(self.program.elements)):
            return
        el = self.program.elements[index]
        if el.element_type == ElementType.WAYPOINT and el.waypoint is not None:
            el.waypoint.name = new_name
            self.program_changed.emit()

    def set_element_scaling(
        self, index: int, velocity_scaling: Optional[float], acceleration_scaling: Optional[float]
    ) -> None:
        """Set (or clear, with None) a waypoint element's own speed
        override -- this is the per-program-waypoint velocity/acceleration
        control. No-op for non-waypoint elements."""
        if not (0 <= index < len(self.program.elements)):
            return
        el = self.program.elements[index]
        if el.element_type != ElementType.WAYPOINT:
            return
        el.velocity_scaling = velocity_scaling
        el.acceleration_scaling = acceleration_scaling
        self.program_changed.emit()

    def delete_element(self, index: int) -> None:
        if self.is_running():
            self.error_occurred.emit('Cannot edit a program while it is running.')
            return
        if 0 <= index < len(self.program.elements):
            el = self.program.elements.pop(index)
            self._element_states.pop(el.element_id, None)
            self.program_changed.emit()

    def move_up(self, index: int) -> None:
        if self.is_running():
            return
        els = self.program.elements
        if 1 <= index < len(els):
            els[index - 1], els[index] = els[index], els[index - 1]
            self.program_changed.emit()

    def move_down(self, index: int) -> None:
        if self.is_running():
            return
        els = self.program.elements
        if 0 <= index < len(els) - 1:
            els[index + 1], els[index] = els[index], els[index + 1]
            self.program_changed.emit()

    # -- persistence --

    def save(self, path: str) -> None:
        save_program(path, self.program)
        _log.info(f"Saved program '{self.program.name}' ({len(self.program.elements)} elements) to {path}")

    def load(self, path: str) -> None:
        """Load a program file into the registry (adding it, or replacing
        the entry of the same name if already present) and make it active."""
        if self.is_running():
            self.error_occurred.emit('Cannot load a program while one is running.')
            return
        program = load_program(path)
        program.loop_count = _normalize_loop_count(program.loop_count)
        self._programs[program.name] = program
        self._active_name = program.name
        self._element_states = {el.element_id: ElementState.PENDING for el in program.elements}
        self._set_execution_state(ExecutionState.IDLE)
        _log.info(f"Loaded program '{program.name}' ({len(program.elements)} elements) from {path}")
        self.programs_changed.emit()
        self.program_changed.emit()

    def load_all_from_directory(self, directory: str) -> None:
        """Bulk-load every saved *.json program in a directory into the
        registry, for populating the tree with everything on disk at
        startup. Individually-broken files are logged and skipped, not
        fatal. Does not disturb an already-in-progress edit; only replaces
        the initial placeholder 'Untitled' program if it's still empty."""
        if not os.path.isdir(directory):
            return
        loaded_any = False
        for fname in sorted(os.listdir(directory)):
            if not fname.endswith('.json'):
                continue
            path = os.path.join(directory, fname)
            try:
                program = load_program(path)
            except Exception as exc:  # noqa: BLE001
                _log.warning(f'Skipping unreadable program file {path}: {exc}')
                continue
            program.loop_count = _normalize_loop_count(program.loop_count)
            self._programs[program.name] = program
            loaded_any = True
        if not loaded_any:
            return
        placeholder = self._programs.get('Untitled')
        if placeholder is not None and not placeholder.elements and len(self._programs) > 1:
            del self._programs['Untitled']
        if self._active_name not in self._programs:
            self._active_name = next(iter(self._programs))
        self._element_states = {el.element_id: ElementState.PENDING for el in self.program.elements}
        self.programs_changed.emit()
        self.program_changed.emit()

    # -- execution --

    def execute_element(self, index: int) -> bool:
        """Run exactly one element (waypoint move or jaw open/close) on its
        own, without running the rest of the program -- for trying out or
        re-checking a single step from the tree. Blocked while a full
        run() (or another single-element execution) is already in
        progress; does not touch any other element's state."""
        if self.is_running():
            self.error_occurred.emit('Cannot execute a single element while the program is running.')
            return False
        if not (0 <= index < len(self.program.elements)):
            return False

        element = self.program.elements[index]
        self._current_index = index
        self._current_request_id = str(uuid.uuid4())
        self._single_element_run = True
        self._set_element_state(element, ElementState.RUNNING)
        self._set_execution_state(ExecutionState.RUNNING)
        self._dispatch_element(element)
        return True

    def run(self) -> None:
        if self.is_running():
            return
        if not self.program.elements:
            self.error_occurred.emit('Program is empty -- nothing to run.')
            return
        for el in self.program.elements:
            self._element_states[el.element_id] = ElementState.PENDING
        self._current_index = -1
        self._current_loop = 1
        self._single_element_run = False
        loop_count = self.program.loop_count
        suffix = 'forever' if loop_count == 0 else f'x{loop_count}'
        _log.info(f"Running program '{self.program.name}' ({len(self.program.elements)} elements, {suffix})")
        self.loop_progress_changed.emit(self._current_loop, loop_count)
        self._set_execution_state(ExecutionState.RUNNING)
        self._advance()

    def stop(self) -> None:
        if not self.is_running():
            return
        _log.info('STOP PROGRAM requested')
        self._ri.request_cancel_all()
        if 0 <= self._current_index < len(self.program.elements):
            # Didn't finish -- back to pending, not left dangling as "running".
            element = self.program.elements[self._current_index]
            self._set_element_state(element, ElementState.PENDING)
        self._single_element_run = False
        self._set_execution_state(ExecutionState.STOPPED)

    def _advance(self) -> None:
        self._current_index += 1
        if self._current_index >= len(self.program.elements):
            loop_count = self.program.loop_count
            if loop_count == 0 or self._current_loop < loop_count:
                # Loop again: infinite (0), or more repeats still owed.
                self._current_loop += 1
                _log.info(f"Program '{self.program.name}' looping (iteration {self._current_loop}"
                          f"{'' if loop_count == 0 else f'/{loop_count}'})")
                self.loop_progress_changed.emit(self._current_loop, loop_count)
                for el in self.program.elements:
                    self._element_states[el.element_id] = ElementState.PENDING
                self._current_index = -1
                self._advance()
                return
            _log.info(f"Program '{self.program.name}' finished ({self._current_loop} loop"
                      f"{'s' if self._current_loop != 1 else ''})")
            self._set_execution_state(ExecutionState.FINISHED)
            return

        element = self.program.elements[self._current_index]
        self._set_element_state(element, ElementState.RUNNING)
        self._current_request_id = str(uuid.uuid4())
        self._dispatch_element(element)

    def _dispatch_element(self, element: ProgramElement) -> None:
        """Send the right command (or start the right timer) for one
        element. Shared by run()'s _advance() and execute_element() --
        both have already set self._current_index/_current_request_id and
        marked the element RUNNING before calling this."""
        if element.element_type == ElementType.WAYPOINT:
            self._execute_waypoint(element)
        elif element.element_type == ElementType.JAW_OPEN:
            self._ri.request_gripper_goal(
                self._gripper_open, self._gripper_max_effort, request_id=self._current_request_id
            )
        elif element.element_type == ElementType.JAW_CLOSE:
            self._ri.request_gripper_goal(
                self._gripper_closed, self._gripper_max_effort, request_id=self._current_request_id
            )
        elif element.element_type == ElementType.WAIT:
            self._execute_wait(element)

    def _execute_wait(self, element: ProgramElement) -> None:
        """No goal sent to the robot at all -- just pauses the sequence
        for wait_duration_sec before advancing. Gated by request_id like
        every other element, so a STOP (or a new execute_element() call)
        before the timer fires correctly makes it a no-op instead of
        resurrecting/advancing a program that's already moved on. Uses an
        instance QTimer (not the QTimer.singleShot() free function) so
        tests can trigger it directly instead of waiting on a real clock,
        same pattern as the arm/gripper streaming timers."""
        duration_ms = max(0, int(element.wait_duration_sec * 1000))
        request_id = self._current_request_id
        self._wait_timer = QTimer(self)
        self._wait_timer.setSingleShot(True)
        self._wait_timer.timeout.connect(lambda: self._on_wait_finished(request_id))
        self._wait_timer.start(duration_ms)

    def _on_wait_finished(self, request_id: str) -> None:
        if self._execution_state != ExecutionState.RUNNING:
            return
        if request_id != self._current_request_id:
            return  # stale -- STOP or a different element started since this was scheduled
        element = self.program.elements[self._current_index]
        self._set_element_state(element, ElementState.COMPLETED)
        if self._single_element_run:
            self._single_element_run = False
            self._set_execution_state(ExecutionState.IDLE)
            return
        self._advance()

    def _execute_waypoint(self, element: ProgramElement) -> None:
        # Arm-only, deliberately: gripper state in a Program is owned
        # exclusively by explicit Jaw Open/Close elements. A waypoint also
        # re-commanding whatever gripper position happened to be captured
        # when it was recorded would silently fight a Jaw element placed
        # elsewhere in the sequence (e.g. a waypoint recorded gripper-open,
        # placed after a Jaw Close element, would re-open it on arrival).
        wp = element.waypoint
        assert wp is not None
        errors = self._joint_limits.validate_positions(wp.joint_names, wp.joint_positions)
        if errors:
            self._fail_current(f"Waypoint '{wp.name}' invalid:\n" + '\n'.join(errors))
            return

        by_name = dict(zip(wp.joint_names, wp.joint_positions))
        arm_positions = [by_name[n] for n in self._arm_joint_names if n in by_name]
        if len(arm_positions) != len(self._arm_joint_names):
            self._fail_current(f"Waypoint '{wp.name}' is missing data for one or more arm joints")
            return

        duration = self._element_duration(wp)
        self._ri.request_arm_goal(
            self._arm_joint_names, arm_positions, duration,
            request_id=self._current_request_id,
        )

    def _element_duration(self, wp: Waypoint) -> float:
        """Same motion-timing rule WaypointManager.move_to() uses for the
        standalone library: the configured base duration, scaled by the
        element's own velocity_scaling (or the app default), but never
        faster than what the joints' (scaled) velocity/acceleration limits
        actually allow."""
        element = self.program.elements[self._current_index]
        velocity_scaling, acceleration_scaling = self.resolved_scaling(element)
        velocity_scaled_duration = max(self._waypoint_duration / max(velocity_scaling, 0.01), 0.1)
        current_positions = [
            self._robot_state.positions.get(name, target)
            for name, target in zip(wp.joint_names, wp.joint_positions)
        ]
        physical_min_duration = self._joint_limits.min_move_duration(
            wp.joint_names, current_positions, wp.joint_positions,
            velocity_scaling=velocity_scaling, acceleration_scaling=acceleration_scaling,
        )
        return max(velocity_scaled_duration, physical_min_duration)

    def _on_trajectory_result(self, outcome: ActionOutcome) -> None:
        if self._execution_state != ExecutionState.RUNNING:
            return
        if outcome.request_id != self._current_request_id:
            return  # a waypoint's "-gripper" sub-goal, or unrelated -- doesn't gate

        if not outcome.accepted:
            self._fail_current(f'Goal rejected: {outcome.message}')
            return

        element = self.program.elements[self._current_index]
        if not outcome.succeeded and element.element_type not in (ElementType.JAW_OPEN, ElementType.JAW_CLOSE):
            self._fail_current(f'Goal did not report success: {outcome.message}')
            return
        if not outcome.succeeded:
            # GripperActionController reports the goal ABORTED (not
            # SUCCEEDED) whenever the gripper stalls before reaching the
            # commanded position within tolerance -- which is exactly what
            # happens when it closes onto an object, or when
            # gripper_closed_position is deliberately set past the
            # mechanical limit to guarantee full closing force. That's the
            # gripper doing its job, not a failure, so it doesn't abort the
            # program -- just logged so it's visible if it ever needs
            # investigating.
            _log.warning(
                f"Program '{self.program.name}': gripper element {self._current_index} "
                f"('{element.display_name}') did not report SUCCEEDED ({outcome.message}) -- "
                f'treating as complete (stall against resistance is expected for a gripper close).'
            )

        self._set_element_state(element, ElementState.COMPLETED)
        if self._single_element_run:
            self._single_element_run = False
            self._set_execution_state(ExecutionState.IDLE)
            return
        self._advance()

    def _fail_current(self, message: str) -> None:
        single = self._single_element_run
        self._single_element_run = False
        if 0 <= self._current_index < len(self.program.elements):
            element = self.program.elements[self._current_index]
            self._set_element_state(element, ElementState.FAILED)
            _log.error(f"Program '{self.program.name}' failed at element {self._current_index} "
                       f"('{element.display_name}'): {message}")
        prefix = 'Element execution failed' if single else 'Program stopped'
        self.error_occurred.emit(f'{prefix}:\n{message}')
        self._set_execution_state(ExecutionState.FAILED)

    def _set_element_state(self, element: ProgramElement, state: ElementState) -> None:
        self._element_states[element.element_id] = state
        self.element_state_changed.emit(self._current_index, state)

    def _set_execution_state(self, state: ExecutionState) -> None:
        self._execution_state = state
        self.execution_state_changed.emit(state)
