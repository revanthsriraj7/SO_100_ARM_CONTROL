"""The Qt <-> ROS bridge. This is the ONLY place rclpy.spin_once() is
called, and the only class that touches the QThread boundary directly.

Threading model:
- RobotInterface subclasses QThread and, in run(), constructs the
  DashboardNode and loops rclpy.spin_once() + draining a thread-safe
  command queue -- so every rclpy call (subscription callbacks, service/
  action callbacks, and outgoing requests) executes on this one worker
  thread, serialized with each other. No two rclpy calls ever race.
- Incoming data (joint states, service/action results) is delivered to
  the rest of the app via Qt Signals emitted from within run() -- Qt
  automatically marshals these to the receiving (GUI) thread's event loop
  via a queued connection, since sender and receiver live in different
  threads.
- Outgoing requests (toggle torque, send a goal, cancel) are plain
  thread-safe methods (request_*) that push a closure onto a queue.Queue;
  the worker thread pops and executes it between spin_once() calls. GUI
  code never touches the rclpy Node directly.
"""
from __future__ import annotations

import queue
import threading
from typing import Optional, Sequence

import rclpy
from PySide6.QtCore import QThread, Signal

from .ros_node import DashboardNode
from ..utils.logging import get_logger
from ..utils.signals import (
    ActionOutcome, ControllerStateSnapshot, JointStateSnapshot, RosbagOutcome, ServiceCallOutcome,
)

_log = get_logger('ros.robot_interface')


class RobotInterface(QThread):
    joint_state_received = Signal(object)       # JointStateSnapshot
    controller_state_received = Signal(object)  # ControllerStateSnapshot
    torque_result = Signal(object)               # ServiceCallOutcome
    gripper_lock_result = Signal(object)          # ServiceCallOutcome
    trajectory_result = Signal(object)            # ActionOutcome
    rosbag_result = Signal(object)                # RosbagOutcome
    node_ready = Signal()
    node_error = Signal(str)

    def __init__(self, robot_config: dict, safety_config: dict, parent=None) -> None:
        super().__init__(parent)
        self._robot_config = robot_config
        self._safety_config = safety_config
        self._command_queue: "queue.Queue" = queue.Queue()
        self._stop_event = threading.Event()
        self._node: Optional[DashboardNode] = None
        self._owns_rclpy_init = False

    # -- QThread.run(): everything below this point executes on the ROS thread --

    def run(self) -> None:
        try:
            if not rclpy.ok():
                rclpy.init(args=None)
                self._owns_rclpy_init = True
        except Exception as exc:  # noqa: BLE001
            self.node_error.emit(f'rclpy.init failed: {exc}')
            return

        try:
            self._node = DashboardNode(
                self._robot_config,
                self._safety_config,
                on_joint_state=lambda s: self.joint_state_received.emit(s),
                on_controller_state=lambda s: self.controller_state_received.emit(s),
            )
        except Exception as exc:  # noqa: BLE001
            _log.error(f'Failed to construct DashboardNode: {exc}')
            self.node_error.emit(str(exc))
            return

        _log.info('ROS worker thread started, node ready')
        self.node_ready.emit()

        while not self._stop_event.is_set():
            rclpy.spin_once(self._node, timeout_sec=0.05)
            self._drain_commands()

        self._node.destroy_node()
        if self._owns_rclpy_init:
            try:
                rclpy.shutdown()
            except Exception:  # noqa: BLE001
                pass
        _log.info('ROS worker thread stopped')

    def _drain_commands(self) -> None:
        while True:
            try:
                job = self._command_queue.get_nowait()
            except queue.Empty:
                break
            try:
                job()
            except Exception as exc:  # noqa: BLE001
                _log.error(f'Queued ROS command raised: {exc}')

    # -- thread-safe public API, called from the Qt main thread --

    def stop(self) -> None:
        self._stop_event.set()

    def request_toggle_torque(self, request_id: Optional[str] = None) -> None:
        def _do() -> None:
            assert self._node is not None
            self._node.call_toggle_torque(
                lambda outcome: self.torque_result.emit(outcome), request_id=request_id
            )
        self._command_queue.put(_do)

    def request_set_gripper_lock(self, locked: bool, request_id: Optional[str] = None) -> None:
        def _do() -> None:
            assert self._node is not None
            self._node.call_set_gripper_lock(
                locked, lambda outcome: self.gripper_lock_result.emit(outcome), request_id=request_id
            )
        self._command_queue.put(_do)

    def request_arm_goal(
        self,
        joint_names: Sequence[str],
        positions: Sequence[float],
        duration_sec: float,
        request_id: Optional[str] = None,
    ) -> None:
        def _do() -> None:
            assert self._node is not None
            self._node.trajectory.send_arm_goal(
                joint_names, positions, duration_sec,
                lambda outcome: self.trajectory_result.emit(outcome),
                request_id=request_id,
            )
        self._command_queue.put(_do)

    def request_gripper_goal(
        self, position: float, max_effort: float, request_id: Optional[str] = None
    ) -> None:
        def _do() -> None:
            assert self._node is not None
            self._node.trajectory.send_gripper_goal(
                position, max_effort,
                lambda outcome: self.trajectory_result.emit(outcome),
                request_id=request_id,
            )
        self._command_queue.put(_do)

    def request_full_trajectory_goal(
        self,
        joint_names: Sequence[str],
        points,
        request_id: Optional[str] = None,
    ) -> None:
        """points: iterable of (time_from_start_sec, positions, velocities_or_None)."""
        def _do() -> None:
            assert self._node is not None
            from builtin_interfaces.msg import Duration
            from trajectory_msgs.msg import JointTrajectoryPoint

            jtc_points = []
            for time_from_start, positions, velocities in points:
                pt = JointTrajectoryPoint()
                pt.positions = list(positions)
                if velocities is not None:
                    pt.velocities = list(velocities)
                sec = int(time_from_start)
                pt.time_from_start = Duration(sec=sec, nanosec=int((time_from_start - sec) * 1e9))
                jtc_points.append(pt)

            self._node.trajectory.send_full_trajectory_goal(
                joint_names, jtc_points,
                lambda outcome: self.trajectory_result.emit(outcome),
                request_id=request_id,
            )
        self._command_queue.put(_do)

    def request_cancel_all(self) -> None:
        def _do() -> None:
            assert self._node is not None
            self._node.trajectory.cancel_all()
            _log.info('Cancelled active trajectory/gripper goals')
        self._command_queue.put(_do)

    def request_start_rosbag(self, topics: Sequence[str], output_dir: str) -> None:
        def _do() -> None:
            assert self._node is not None
            success, message = self._node.rosbag.start(list(topics), output_dir)
            self.rosbag_result.emit(
                RosbagOutcome(success=success, message=message, recording=self._node.rosbag.is_recording)
            )
        self._command_queue.put(_do)

    def request_stop_rosbag(self) -> None:
        def _do() -> None:
            assert self._node is not None
            success, message = self._node.rosbag.stop()
            self.rosbag_result.emit(
                RosbagOutcome(success=success, message=message, recording=self._node.rosbag.is_recording)
            )
        self._command_queue.put(_do)
