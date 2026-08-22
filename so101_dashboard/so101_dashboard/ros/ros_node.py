"""The rclpy Node itself: pure ROS, no Qt. Composes the joint-state
subscriber, trajectory/gripper action interface, and the /toggle_torque
and /set_gripper_lock service clients.

Kept independent of Qt so it can be constructed and driven (via
rclpy.spin_once) in a plain Python script or test, with no GUI involved --
robot_interface.py is the only place that wires this into Qt.
"""
from __future__ import annotations

from typing import Callable, Optional

from rclpy.node import Node
from rclpy.task import Future
from std_srvs.srv import SetBool, Trigger

from .joint_state_subscriber import JointStateSubscriber
from .trajectory_controller import TrajectoryController
from ..recording.rosbag_recorder import RosbagRecorder
from ..utils.signals import ControllerStateSnapshot, JointStateSnapshot, ServiceCallOutcome


class DashboardNode(Node):
    def __init__(
        self,
        robot_config: dict,
        safety_config: dict,
        on_joint_state: Callable[[JointStateSnapshot], None],
        on_controller_state: Optional[Callable[[ControllerStateSnapshot], None]] = None,
        node_name: str = 'so101_dashboard',
    ) -> None:
        super().__init__(node_name)

        self.joint_states_subscriber = JointStateSubscriber(
            self,
            joint_state_topic=robot_config['joint_state_topic'],
            controller_state_topic=robot_config['controller_state_topic'],
            on_joint_state=on_joint_state,
            on_controller_state=on_controller_state,
        )

        self.trajectory = TrajectoryController(
            self,
            trajectory_topic=robot_config['trajectory_topic'],
            follow_joint_trajectory_action=robot_config['follow_joint_trajectory_action'],
            gripper_command_action=robot_config['gripper_command_action'],
        )

        self._torque_client = self.create_client(Trigger, robot_config['torque_service'])
        self._torque_timeout = safety_config.get('torque_service_timeout', 2.0)

        self._gripper_lock_client = self.create_client(
            SetBool, robot_config.get('gripper_lock_service', '/set_gripper_lock')
        )

        self.rosbag = RosbagRecorder(self)

    def call_toggle_torque(
        self, on_done: Callable[[ServiceCallOutcome], None], request_id: Optional[str] = None
    ) -> None:
        if not self._torque_client.service_is_ready():
            self.get_logger().error(
                f"Torque service '{self._torque_client.srv_name}' not available"
            )
            on_done(ServiceCallOutcome(
                success=False,
                message=f"Torque service not available",
                request_id=request_id,
            ))
            return

        self.get_logger().info('Calling /toggle_torque')
        future = self._torque_client.call_async(Trigger.Request())

        def _on_response(fut: Future) -> None:
            try:
                result = fut.result()
            except Exception as exc:  # noqa: BLE001 - report any client-side failure
                self.get_logger().error(f'Torque service call failed: {exc}')
                on_done(ServiceCallOutcome(
                    success=False, message=f'Service call failed: {exc}', request_id=request_id
                ))
                return

            if result is None:
                on_done(ServiceCallOutcome(
                    success=False, message='No response from torque service',
                    request_id=request_id,
                ))
                return

            self.get_logger().info(f'Torque service response: {result.message}')
            on_done(ServiceCallOutcome(
                success=result.success, message=result.message, request_id=request_id
            ))

        future.add_done_callback(_on_response)

    def call_set_gripper_lock(
        self, locked: bool, on_done: Callable[[ServiceCallOutcome], None], request_id: Optional[str] = None
    ) -> None:
        if not self._gripper_lock_client.service_is_ready():
            self.get_logger().error(
                f"Gripper lock service '{self._gripper_lock_client.srv_name}' not available"
            )
            on_done(ServiceCallOutcome(
                success=False,
                message='Gripper lock service not available',
                request_id=request_id,
            ))
            return

        self.get_logger().info(f'Calling /set_gripper_lock({locked})')
        future = self._gripper_lock_client.call_async(SetBool.Request(data=locked))

        def _on_response(fut: Future) -> None:
            try:
                result = fut.result()
            except Exception as exc:  # noqa: BLE001 - report any client-side failure
                self.get_logger().error(f'Gripper lock service call failed: {exc}')
                on_done(ServiceCallOutcome(
                    success=False, message=f'Service call failed: {exc}', request_id=request_id
                ))
                return

            if result is None:
                on_done(ServiceCallOutcome(
                    success=False, message='No response from gripper lock service',
                    request_id=request_id,
                ))
                return

            self.get_logger().info(f'Gripper lock service response: {result.message}')
            on_done(ServiceCallOutcome(
                success=result.success, message=result.message, request_id=request_id
            ))

        future.add_done_callback(_on_response)
