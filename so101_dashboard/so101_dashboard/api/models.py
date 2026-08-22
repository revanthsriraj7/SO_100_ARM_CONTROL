"""Pydantic request/response schemas for the REST API.

These are presentation-layer shapes only -- built fresh from the real
managers on every request (see routes/*.py), never persisted or held
anywhere. Nothing here is a second copy of program/target/robot state;
it's just how the existing in-memory state gets serialized to JSON.
"""
from __future__ import annotations

from typing import Dict, List, Optional

from pydantic import BaseModel, Field


class ErrorResponse(BaseModel):
    error: str = Field(..., description="Machine-readable error code, e.g. 'PROGRAM_NOT_FOUND'.")
    message: str = Field(..., description='Human-readable explanation.')


# -- robot / torque --

class ProgramStateInfo(BaseModel):
    name: Optional[str] = Field(None, description='Active program name, if any.')
    state: str = Field(..., description='IDLE | RUNNING | STOPPED | FINISHED | FAILED')
    current_element_index: Optional[int] = Field(None, description='0-based index of the element currently running, if any.')
    current_element_name: Optional[str] = None
    current_loop: Optional[int] = Field(None, description='1-based current loop iteration.')
    loop_count: Optional[int] = Field(None, description='0 = loop forever; otherwise total loops.')


class RobotStateResponse(BaseModel):
    connected: bool = Field(..., description='True if the ROS 2 node is up and joint states are flowing and fresh.')
    torque_state: str = Field(..., description='Raw TorqueManager state name (see /api/robot/torque).')
    joints: Dict[str, float] = Field(..., description='Actual joint feedback from /joint_states -- not commanded values.')
    gripper: Dict[str, float] = Field(..., description="The gripper joint's actual position, keyed by joint name.")
    program: ProgramStateInfo


class RobotStatusResponse(BaseModel):
    ros_connected: bool
    joint_states_receiving: bool
    joint_states_stale: bool
    controller_active: bool
    mock_mode: bool = Field(..., description='True if the dashboard is running in --mock (no real hardware).')


class TorqueStateResponse(BaseModel):
    state: str = Field(..., description='One of: UNKNOWN, TORQUE_ENABLED, TORQUE_DISABLED, TOGGLING, FREE_MOVE, CAPTURING_POSITION, PREPARING_HOLD, HOLDING, ERROR')
    can_enable: bool = Field(..., description='Whether POST /api/robot/torque/enable would currently be accepted.')
    can_disable: bool = Field(..., description='Whether POST /api/robot/torque/disable would currently be accepted.')
    gripper_lock_enabled: bool = Field(..., description='Whether the gripper keeps torque during free-move.')


class TorqueActionResponse(BaseModel):
    accepted: bool
    state: str = Field(..., description='TorqueManager state after the action was requested.')
    message: str = ''


# -- targets --

class TargetSummary(BaseModel):
    name: str
    joint_names: List[str]


class TargetDetail(BaseModel):
    name: str
    joint_names: List[str]
    joints: Dict[str, float] = Field(..., description='joint_name -> position (radians), the saved target pose.')
    cartesian_pose: Optional[Dict[str, float]] = Field(
        None, description="Forward-kinematics {x, y, z} of the arm joints in this target, if computable."
    )


class MoveToTargetRequest(BaseModel):
    duration_sec: float = Field(3.0, gt=0, description='Nominal move duration; may be lengthened to respect joint speed limits.')
    velocity_scaling: float = Field(1.0, gt=0, le=1.0)
    acceleration_scaling: float = Field(1.0, gt=0, le=1.0)
    wait: bool = Field(True, description='If true, block until the move completes (or times out) before responding.')


class MoveResultResponse(BaseModel):
    accepted: bool
    succeeded: Optional[bool] = None
    message: str = ''


# -- programs --

class ProgramElementResponse(BaseModel):
    index: int
    type: str = Field(..., description='waypoint | jaw_open | jaw_close | wait')
    name: str
    state: str = Field(..., description='PENDING | RUNNING | COMPLETED | FAILED')
    wait_duration_sec: Optional[float] = None
    velocity_scaling: Optional[float] = None
    acceleration_scaling: Optional[float] = None


class ProgramSummary(BaseModel):
    name: str
    num_elements: int
    loop_count: int = Field(..., description='0 = loop forever; otherwise total loops per run.')
    is_active: bool = Field(..., description='Whether this is the dashboard UI\'s currently-selected program.')


class ProgramDetail(BaseModel):
    name: str
    elements: List[ProgramElementResponse]
    loop_count: int
    execution_state: str
    current_element_index: Optional[int] = None
    current_loop: Optional[int] = None


class RunProgramResponse(BaseModel):
    accepted: bool
    program: str
    execution_state: str
    message: str = ''


class StopProgramResponse(BaseModel):
    execution_state: str
    message: str = ''


class CreateProgramRequest(BaseModel):
    name: str = Field(..., min_length=1, max_length=200)


class AddTargetElementRequest(BaseModel):
    target_name: str
    index: Optional[int] = Field(None, description='Insert position; omit to append.')
    velocity_scaling: Optional[float] = Field(None, gt=0, le=1.0)
    acceleration_scaling: Optional[float] = Field(None, gt=0, le=1.0)


# -- gripper --

class GripperCommandRequest(BaseModel):
    wait: bool = Field(True, description='If true, block until the gripper goal completes before responding.')


class GripperPositionRequest(BaseModel):
    position: float = Field(..., description='Target gripper joint position, radians.')
    max_effort: Optional[float] = Field(None, gt=0)
    wait: bool = True


class GripperStateResponse(BaseModel):
    position: float
    accepted: bool
    succeeded: Optional[bool] = None
    message: str = ''
