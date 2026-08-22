"""Shared pytest fixtures: a QApplication (required for any QObject with
QTimers to function, even headless) and a FakeRobotInterface test double
matching RobotInterface's public signal/method surface, so
TorqueManager/WaypointManager/TrajectoryManager can be unit tested without
rclpy or a real ROS graph -- exactly what the project spec asks for.
"""
import os
import sys

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

import pytest
from PySide6.QtCore import QObject, Signal
from PySide6.QtWidgets import QApplication

from so101_dashboard.utils.signals import ActionOutcome, ServiceCallOutcome


@pytest.fixture(scope='session')
def qapp():
    app = QApplication.instance() or QApplication(sys.argv)
    yield app


class FakeRobotInterface(QObject):
    """Records every request_* call so tests can assert on what was sent,
    and lets tests manually emit results to simulate async ROS responses
    arriving -- the tests control exactly when/whether a response comes
    back, which is what makes the "service failure" / "timeout" /
    "double click" test cases possible without a real service.
    """
    joint_state_received = Signal(object)
    controller_state_received = Signal(object)
    torque_result = Signal(object)
    gripper_lock_result = Signal(object)
    trajectory_result = Signal(object)
    rosbag_result = Signal(object)
    node_ready = Signal()
    node_error = Signal(str)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.toggle_torque_calls = []
        self.gripper_lock_calls = []
        self.arm_goal_calls = []
        self.gripper_goal_calls = []
        self.cancel_all_calls = 0

    def request_toggle_torque(self, request_id=None):
        self.toggle_torque_calls.append(request_id)

    def request_set_gripper_lock(self, locked, request_id=None):
        self.gripper_lock_calls.append((locked, request_id))

    def request_arm_goal(self, joint_names, positions, duration_sec, request_id=None):
        self.arm_goal_calls.append((list(joint_names), list(positions), duration_sec, request_id))

    def request_gripper_goal(self, position, max_effort, request_id=None):
        self.gripper_goal_calls.append((position, max_effort, request_id))

    def request_full_trajectory_goal(self, joint_names, points, request_id=None):
        self.arm_goal_calls.append((list(joint_names), list(points), None, request_id))

    def request_cancel_all(self):
        self.cancel_all_calls += 1

    # -- test helpers to simulate async responses --

    def succeed_last_toggle(self, message='ok'):
        request_id = self.toggle_torque_calls[-1]
        self.torque_result.emit(ServiceCallOutcome(success=True, message=message, request_id=request_id))

    def fail_last_toggle(self, message='simulated failure'):
        request_id = self.toggle_torque_calls[-1]
        self.torque_result.emit(ServiceCallOutcome(success=False, message=message, request_id=request_id))

    def succeed_last_gripper_lock(self, message='ok'):
        _locked, request_id = self.gripper_lock_calls[-1]
        self.gripper_lock_result.emit(ServiceCallOutcome(success=True, message=message, request_id=request_id))

    def fail_last_gripper_lock(self, message='simulated failure'):
        _locked, request_id = self.gripper_lock_calls[-1]
        self.gripper_lock_result.emit(ServiceCallOutcome(success=False, message=message, request_id=request_id))

    def succeed_last_arm_goal(self, message='ok'):
        request_id = self.arm_goal_calls[-1][3]
        self.trajectory_result.emit(ActionOutcome(accepted=True, succeeded=True, message=message, request_id=request_id))

    def reject_last_arm_goal(self, message='rejected'):
        request_id = self.arm_goal_calls[-1][3]
        self.trajectory_result.emit(ActionOutcome(accepted=False, succeeded=False, message=message, request_id=request_id))


@pytest.fixture
def fake_ri(qapp):
    return FakeRobotInterface()
