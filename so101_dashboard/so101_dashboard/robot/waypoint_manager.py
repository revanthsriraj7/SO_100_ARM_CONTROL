"""Owns the in-memory waypoint list plus RECORD / MOVE TO, delegating
motion to RobotInterface (validated against JointLimits first) and
persistence to storage/waypoint_storage.py.
"""
from __future__ import annotations

import uuid
from typing import List, Optional

from PySide6.QtCore import QObject, Signal

from .joint_limits import JointLimits
from .robot_state import RobotState
from ..ros.robot_interface import RobotInterface
from ..storage.waypoint_storage import Waypoint, load_waypoints, save_waypoints
from ..utils.logging import get_logger
from ..utils.signals import ActionOutcome

_log = get_logger('robot.waypoint_manager')


class WaypointManager(QObject):
    waypoints_changed = Signal()
    move_started = Signal(str)     # waypoint name
    move_finished = Signal(object)  # ActionOutcome
    error_occurred = Signal(str)

    def __init__(
        self,
        robot_interface: RobotInterface,
        robot_state: RobotState,
        joint_limits: JointLimits,
        arm_joint_names: List[str],
        gripper_joint_name: str,
        parent: Optional[QObject] = None,
    ) -> None:
        super().__init__(parent)
        self._ri = robot_interface
        self._robot_state = robot_state
        self._joint_limits = joint_limits
        self._arm_joint_names = list(arm_joint_names)
        self._gripper_joint_name = gripper_joint_name
        self._all_names = self._arm_joint_names + [gripper_joint_name]
        self._waypoints: List[Waypoint] = []
        self._move_request_id = ''

        robot_interface.trajectory_result.connect(self._on_trajectory_result)

    @property
    def waypoints(self) -> List[Waypoint]:
        return list(self._waypoints)

    # -- editing --

    def record(self, name: str) -> bool:
        rs = self._robot_state
        missing = [n for n in self._all_names if n not in rs.positions]
        if missing:
            self.error_occurred.emit(f'Cannot record waypoint: no data yet for {missing}')
            return False
        positions = [rs.positions[n] for n in self._all_names]
        self._waypoints.append(Waypoint.create(name, self._all_names, positions))
        _log.info(f"Recorded waypoint '{name}': {dict(zip(self._all_names, positions))}")
        self.waypoints_changed.emit()
        return True

    def add_existing(self, waypoint: Waypoint) -> bool:
        """Add a copy of a waypoint that came from somewhere other than
        RECORD (e.g. auto-imported out of a loaded Program file) --
        skipped, not duplicated, if a waypoint with the same name is
        already in the library. Returns True if it was actually added."""
        if any(wp.name == waypoint.name for wp in self._waypoints):
            return False
        self._waypoints.append(Waypoint.create(waypoint.name, waypoint.joint_names, waypoint.joint_positions))
        self.waypoints_changed.emit()
        return True

    def update_or_add(self, waypoint: Waypoint) -> None:
        """Like add_existing(), but OVERWRITES an existing library entry
        with the same name instead of skipping it -- used when a
        Program's copy of a waypoint gets re-recorded, so the library
        (which may well be where it originally came from) reflects the
        correction too, not just the program."""
        for i, wp in enumerate(self._waypoints):
            if wp.name == waypoint.name:
                self._waypoints[i] = Waypoint.create(waypoint.name, waypoint.joint_names, waypoint.joint_positions)
                self.waypoints_changed.emit()
                return
        self._waypoints.append(Waypoint.create(waypoint.name, waypoint.joint_names, waypoint.joint_positions))
        self.waypoints_changed.emit()

    def rename(self, index: int, new_name: str) -> None:
        if 0 <= index < len(self._waypoints):
            self._waypoints[index].name = new_name
            self.waypoints_changed.emit()

    def delete(self, index: int) -> None:
        if 0 <= index < len(self._waypoints):
            del self._waypoints[index]
            self.waypoints_changed.emit()

    def duplicate(self, index: int) -> None:
        if 0 <= index < len(self._waypoints):
            wp = self._waypoints[index]
            copy = Waypoint.create(f'{wp.name}_copy', wp.joint_names, wp.joint_positions)
            self._waypoints.insert(index + 1, copy)
            self.waypoints_changed.emit()

    def move_item_up(self, index: int) -> None:
        if 1 <= index < len(self._waypoints):
            self._waypoints[index - 1], self._waypoints[index] = self._waypoints[index], self._waypoints[index - 1]
            self.waypoints_changed.emit()

    def move_item_down(self, index: int) -> None:
        if 0 <= index < len(self._waypoints) - 1:
            self._waypoints[index + 1], self._waypoints[index] = self._waypoints[index], self._waypoints[index + 1]
            self.waypoints_changed.emit()

    def clear(self) -> None:
        self._waypoints.clear()
        self.waypoints_changed.emit()

    # -- persistence --

    def save(self, path: str) -> None:
        save_waypoints(path, self._waypoints)
        _log.info(f'Saved {len(self._waypoints)} waypoints to {path}')

    def load(self, path: str) -> None:
        self._waypoints = load_waypoints(path)
        _log.info(f'Loaded {len(self._waypoints)} waypoints from {path}')
        self.waypoints_changed.emit()

    # -- playback --

    def move_to(
        self,
        index: int,
        duration_sec: float,
        velocity_scaling: float = 1.0,
        acceleration_scaling: float = 1.0,
    ) -> None:
        if not (0 <= index < len(self._waypoints)):
            return
        wp = self._waypoints[index]
        errors = self._joint_limits.validate_positions(wp.joint_names, wp.joint_positions)
        if errors:
            self.error_occurred.emit('Cannot move to waypoint:\n' + '\n'.join(errors))
            return

        by_name = dict(zip(wp.joint_names, wp.joint_positions))

        # Never faster than velocity scaling alone would give (existing
        # behavior), and never faster than the joints' (scaled) velocity/
        # acceleration limits actually allow -- acceleration_scaling only
        # ever adds time (gentler ramps), it can't shorten a move.
        velocity_scaled_duration = max(duration_sec / max(velocity_scaling, 0.01), 0.1)
        current_positions = [
            self._robot_state.positions.get(name, target)
            for name, target in zip(wp.joint_names, wp.joint_positions)
        ]
        physical_min_duration = self._joint_limits.min_move_duration(
            wp.joint_names, current_positions, wp.joint_positions,
            velocity_scaling=velocity_scaling, acceleration_scaling=acceleration_scaling,
        )
        scaled_duration = max(velocity_scaled_duration, physical_min_duration)

        self._move_request_id = str(uuid.uuid4())
        self.move_started.emit(wp.name)
        arm_positions = [by_name[n] for n in self._arm_joint_names if n in by_name]
        if len(arm_positions) == len(self._arm_joint_names):
            self._ri.request_arm_goal(
                self._arm_joint_names, arm_positions, scaled_duration, request_id=self._move_request_id
            )
        if self._gripper_joint_name in by_name:
            self._ri.request_gripper_goal(
                by_name[self._gripper_joint_name], 50.0,
                request_id=f'{self._move_request_id}-gripper',
            )

    def stop(self) -> None:
        self._ri.request_cancel_all()

    def _on_trajectory_result(self, outcome: ActionOutcome) -> None:
        if outcome.request_id == self._move_request_id:
            self.move_finished.emit(outcome)
