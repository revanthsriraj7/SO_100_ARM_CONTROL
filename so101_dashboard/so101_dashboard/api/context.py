"""Everything a route handler needs to reach the real dashboard state --
constructed once in application.py and stashed on `app.state.ctx`. Holds
references only; the API never owns a copy of anything here.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import List

from .bridge import DashboardBridge
from ..robot.gripper_manager import GripperManager
from ..robot.joint_control_manager import JointControlManager
from ..robot.joint_limits import JointLimits
from ..robot.program_manager import ProgramManager
from ..robot.robot_state import RobotState
from ..robot.torque_manager import TorqueManager
from ..robot.waypoint_manager import WaypointManager


@dataclass
class ApiContext:
    bridge: DashboardBridge
    robot_state: RobotState
    torque_manager: TorqueManager
    waypoint_manager: WaypointManager
    program_manager: ProgramManager
    gripper_manager: GripperManager
    joint_control_manager: JointControlManager
    joint_limits: JointLimits
    arm_joint_names: List[str]
    gripper_joint_name: str
    mock_mode: bool
