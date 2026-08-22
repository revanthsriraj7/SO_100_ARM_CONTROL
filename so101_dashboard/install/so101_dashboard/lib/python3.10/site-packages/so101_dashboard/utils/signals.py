"""Plain-data payloads carried across the ROS-worker-thread -> Qt-main-thread
boundary via Signal(object).

These are intentionally plain dataclasses with no ROS or Qt imports: the
ROS worker thread constructs them from ROS messages, Qt's queued
connections marshal them across the thread boundary, and GUI code reads
them without ever touching a ROS message type directly. Keeping ROS
message types out of the GUI layer means widgets can be unit tested
without rclpy.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Optional


@dataclass(frozen=True)
class JointStateSnapshot:
    """One /joint_states message, boiled down to what the app needs."""
    stamp_sec: float                    # monotonic-ish ROS time, seconds
    positions: Dict[str, float] = field(default_factory=dict)
    velocities: Dict[str, float] = field(default_factory=dict)
    efforts: Dict[str, float] = field(default_factory=dict)


@dataclass(frozen=True)
class ControllerStateSnapshot:
    """One JointTrajectoryControllerState message (desired/actual/error)."""
    stamp_sec: float
    joint_names: List[str]
    desired_positions: List[float]
    actual_positions: List[float]
    error_positions: List[float]


@dataclass(frozen=True)
class ServiceCallOutcome:
    """Result of an async service call, delivered back to the requester."""
    success: bool
    message: str
    request_id: Optional[str] = None


@dataclass(frozen=True)
class ActionOutcome:
    """Result of an action goal (FollowJointTrajectory, GripperCommand, ...)."""
    accepted: bool
    succeeded: bool
    message: str
    request_id: Optional[str] = None


@dataclass(frozen=True)
class RosbagOutcome:
    success: bool
    message: str
    recording: bool   # true if a bag is now (still) recording after this call


@dataclass(frozen=True)
class ConnectionStatus:
    ros_connected: bool
    joint_states_receiving: bool
    joint_states_stale: bool
    controller_active: bool
