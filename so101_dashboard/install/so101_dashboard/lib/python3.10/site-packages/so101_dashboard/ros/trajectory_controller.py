"""Sends motion commands to arm_controller / gripper_controller.

Deliberately action-based (FollowJointTrajectory, GripperCommand) rather
than raw topic-publish for every motion that matters (HOLD, HOME, waypoint
moves, trajectory playback): actions give goal-accepted/result feedback,
which TorqueManager and the motion widgets need to confirm a command
actually landed rather than assuming a fire-and-forget publish worked.

A raw topic publisher is also exposed for cases where that confirmation
doesn't matter (e.g. streaming teleop-style commands), but nothing safety
-critical in this app uses it.

No Qt here -- plain rclpy.
"""
from __future__ import annotations

from typing import Callable, List, Optional, Sequence

from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from .action_interface import ActionClientWrapper
from ..utils.signals import ActionOutcome


def _duration(seconds: float) -> Duration:
    sec = int(seconds)
    nanosec = int((seconds - sec) * 1e9)
    return Duration(sec=sec, nanosec=nanosec)


def build_single_point_trajectory(
    joint_names: Sequence[str],
    positions: Sequence[float],
    duration_sec: float,
    velocities: Optional[Sequence[float]] = None,
) -> JointTrajectory:
    traj = JointTrajectory()
    traj.joint_names = list(joint_names)
    point = JointTrajectoryPoint()
    point.positions = list(positions)
    if velocities is not None:
        point.velocities = list(velocities)
    point.time_from_start = _duration(duration_sec)
    traj.points = [point]
    return traj


class TrajectoryController:
    def __init__(
        self,
        node: Node,
        trajectory_topic: str,
        follow_joint_trajectory_action: str,
        gripper_command_action: str,
    ) -> None:
        self._node = node
        self._topic_pub = node.create_publisher(JointTrajectory, trajectory_topic, 10)
        self._arm_action = ActionClientWrapper(
            node, FollowJointTrajectory, follow_joint_trajectory_action
        )
        self._gripper_action = ActionClientWrapper(
            node, GripperCommand, gripper_command_action
        )

    # -- confirmable, action-based (used for HOLD / HOME / waypoints / playback) --

    def send_arm_goal(
        self,
        joint_names: Sequence[str],
        positions: Sequence[float],
        duration_sec: float,
        on_result: Callable[[ActionOutcome], None],
        request_id: Optional[str] = None,
    ) -> None:
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = build_single_point_trajectory(joint_names, positions, duration_sec)
        self._arm_action.send_goal(goal, on_result, request_id=request_id)

    def send_full_trajectory_goal(
        self,
        joint_names: Sequence[str],
        points: List[JointTrajectoryPoint],
        on_result: Callable[[ActionOutcome], None],
        request_id: Optional[str] = None,
        feedback_callback=None,
    ) -> None:
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(joint_names)
        goal.trajectory.points = points
        self._arm_action.send_goal(
            goal, on_result, request_id=request_id, feedback_callback=feedback_callback
        )

    def send_gripper_goal(
        self,
        position: float,
        max_effort: float,
        on_result: Callable[[ActionOutcome], None],
        request_id: Optional[str] = None,
    ) -> None:
        goal = GripperCommand.Goal()
        goal.command.position = position
        goal.command.max_effort = max_effort
        self._gripper_action.send_goal(goal, on_result, request_id=request_id)

    def cancel_arm_goal(self) -> bool:
        return self._arm_action.cancel_active_goal()

    def cancel_gripper_goal(self) -> bool:
        return self._gripper_action.cancel_active_goal()

    def cancel_all(self) -> None:
        self.cancel_arm_goal()
        self.cancel_gripper_goal()

    @property
    def arm_has_active_goal(self) -> bool:
        return self._arm_action.has_active_goal

    # -- fire-and-forget, topic-based (not used for safety-critical motion) --

    def publish_trajectory(
        self, joint_names: Sequence[str], positions: Sequence[float], duration_sec: float
    ) -> None:
        self._topic_pub.publish(
            build_single_point_trajectory(joint_names, positions, duration_sec)
        )
