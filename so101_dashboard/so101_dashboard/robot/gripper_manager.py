"""One-shot gripper open/close/position control, independent of a running
Program. Wraps RobotInterface.request_gripper_goal() directly -- the same
underlying call ProgramManager's JAW_OPEN/JAW_CLOSE elements and
TorqueManager's HOLD path already use -- using the SAME config-driven
gripper_open_position / gripper_closed_position / gripper_hold_max_effort
values, so there is exactly one definition of "open" and "closed" anywhere
in the app, not a second one invented here.

This exists because no standalone "just open/close the gripper right now"
entry point exists outside a Program's internal element dispatch -- the
API layer needs one, and any future UI affordance for the same thing
should reuse this rather than duplicating it again.
"""
from __future__ import annotations

import uuid
from typing import Optional

from PySide6.QtCore import QObject, Signal

from .joint_limits import JointLimits
from ..ros.robot_interface import RobotInterface
from ..utils.logging import get_logger
from ..utils.signals import ActionOutcome

_log = get_logger('robot.gripper_manager')


class GripperManager(QObject):
    goal_finished = Signal(object)   # ActionOutcome
    error_occurred = Signal(str)

    def __init__(
        self,
        robot_interface: RobotInterface,
        joint_limits: JointLimits,
        gripper_joint_name: str,
        motion_config: dict,
        parent: Optional[QObject] = None,
    ) -> None:
        super().__init__(parent)
        self._ri = robot_interface
        self._joint_limits = joint_limits
        self._gripper_joint_name = gripper_joint_name
        self._open_position = float(motion_config.get('gripper_open_position', 1.57))
        self._closed_position = float(motion_config.get('gripper_closed_position', 0.0))
        self._max_effort = float(motion_config.get('gripper_hold_max_effort', 50.0))
        self._request_id = ''

        robot_interface.trajectory_result.connect(self._on_trajectory_result)

    @property
    def open_position(self) -> float:
        return self._open_position

    @property
    def closed_position(self) -> float:
        return self._closed_position

    def open(self) -> str:
        return self._send(self._open_position)

    def close(self) -> str:
        return self._send(self._closed_position)

    def move_to(self, position: float, max_effort: Optional[float] = None) -> str:
        """Command an arbitrary gripper position (validated against
        JointLimits, same as every other motion command in this app)."""
        error = self._joint_limits.validate_position(self._gripper_joint_name, position)
        if error:
            self.error_occurred.emit(error)
            raise ValueError(error)
        return self._send(position, max_effort)

    def _send(self, position: float, max_effort: Optional[float] = None) -> str:
        self._request_id = str(uuid.uuid4())
        _log.info(f'Gripper goal requested: position={position} request_id={self._request_id}')
        self._ri.request_gripper_goal(
            position, max_effort if max_effort is not None else self._max_effort,
            request_id=self._request_id,
        )
        return self._request_id

    def _on_trajectory_result(self, outcome: ActionOutcome) -> None:
        if outcome.request_id != self._request_id:
            return
        self.goal_finished.emit(outcome)
