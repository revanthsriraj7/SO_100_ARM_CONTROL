"""Subscribes to /joint_states and the controller state topic, converting
ROS messages into the plain dataclasses from utils/signals.py.

No Qt here -- this is plain rclpy, usable/testable headless.
"""
from __future__ import annotations

from typing import Callable, Optional

from control_msgs.msg import JointTrajectoryControllerState
from rclpy.node import Node
from sensor_msgs.msg import JointState

from ..utils.signals import ControllerStateSnapshot, JointStateSnapshot


class JointStateSubscriber:
    def __init__(
        self,
        node: Node,
        joint_state_topic: str,
        controller_state_topic: str,
        on_joint_state: Callable[[JointStateSnapshot], None],
        on_controller_state: Optional[Callable[[ControllerStateSnapshot], None]] = None,
    ) -> None:
        self._on_joint_state = on_joint_state
        self._on_controller_state = on_controller_state

        node.create_subscription(JointState, joint_state_topic, self._joint_state_cb, 50)
        if on_controller_state is not None:
            node.create_subscription(
                JointTrajectoryControllerState,
                controller_state_topic,
                self._controller_state_cb,
                50,
            )

    def _joint_state_cb(self, msg: JointState) -> None:
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        positions = dict(zip(msg.name, msg.position)) if msg.position else {}
        velocities = dict(zip(msg.name, msg.velocity)) if msg.velocity else {}
        efforts = dict(zip(msg.name, msg.effort)) if msg.effort else {}
        self._on_joint_state(
            JointStateSnapshot(
                stamp_sec=stamp,
                positions=positions,
                velocities=velocities,
                efforts=efforts,
            )
        )

    def _controller_state_cb(self, msg: JointTrajectoryControllerState) -> None:
        if self._on_controller_state is None:
            return
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self._on_controller_state(
            ControllerStateSnapshot(
                stamp_sec=stamp,
                joint_names=list(msg.joint_names),
                desired_positions=list(msg.desired.positions) if msg.desired.positions else [],
                actual_positions=list(msg.actual.positions) if msg.actual.positions else [],
                error_positions=list(msg.error.positions) if msg.error.positions else [],
            )
        )
