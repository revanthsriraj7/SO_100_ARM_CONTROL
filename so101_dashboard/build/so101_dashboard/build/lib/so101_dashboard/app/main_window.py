"""Top-level window: assembles all tabs -- CONTROL, JOINT CONTROL,
TELEMETRY, PROGRAMS (the merged waypoint-library + program-sequencing
tab), TRAJECTORIES, ROSBAG, SETTINGS.
"""
from __future__ import annotations

from typing import Dict, List

from PySide6.QtCore import Qt
from PySide6.QtWidgets import (
    QHBoxLayout, QLabel, QMainWindow, QTabWidget, QVBoxLayout, QWidget,
)

from ..plotting.plot_manager import PlotManager
from ..recording.trajectory_recorder import TrajectoryRecorder
from ..robot.joint_control_manager import JointControlManager
from ..robot.joint_limits import JointLimits
from ..robot.program_manager import ProgramManager
from ..robot.robot_state import RobotState
from ..robot.torque_manager import TorqueManager
from ..robot.trajectory_manager import TrajectoryManager
from ..robot.waypoint_manager import WaypointManager
from ..ros.robot_interface import RobotInterface
from ..widgets.control_panel import ControlPanel
from ..widgets.joint_control_widget import JointControlWidget
from ..widgets.joint_table import JointTable
from ..widgets.program_widget import ProgramWidget
from ..widgets.rosbag_widget import RosbagWidget
from ..widgets.status_widget import StatusWidget
from ..widgets.telemetry_widget import TelemetryWidget
from ..widgets.trajectory_widget import TrajectoryWidget


def _settings_tab(config_dir: str) -> QWidget:
    w = QWidget()
    layout = QVBoxLayout(w)
    label = QLabel(
        'Settings are file-based -- edit and restart the dashboard to apply:\n\n'
        f'  config.yaml        (topics, rates, storage paths)\n'
        f'  joint_limits.yaml  (per-joint position/velocity/effort limits)\n'
        f'  home_position.yaml (MOVE HOME target)\n\n'
        f'Current config directory:\n{config_dir}'
    )
    label.setAlignment(Qt.AlignCenter)
    label.setStyleSheet('color: #8b949e; font-size: 13px;')
    layout.addWidget(label)
    return w


class MainWindow(QMainWindow):
    def __init__(
        self,
        robot_interface: RobotInterface,
        robot_state: RobotState,
        torque_manager: TorqueManager,
        joint_limits: JointLimits,
        plot_manager: PlotManager,
        waypoint_manager: WaypointManager,
        program_manager: ProgramManager,
        joint_control_manager: JointControlManager,
        trajectory_recorder: TrajectoryRecorder,
        trajectory_manager: TrajectoryManager,
        arm_joint_names: List[str],
        gripper_joint_name: str,
        all_joint_names: List[str],
        home_position: Dict[str, float],
        home_duration_sec: float,
        telemetry_update_rate: float,
        telemetry_history_seconds: float,
        waypoint_directory: str,
        program_directory: str,
        trajectory_directory: str,
        rosbag_directory: str,
        rosbag_available_topics: List[str],
        rosbag_default_topics: List[str],
        config_dir: str,
        mock_mode: bool,
        parent=None,
    ) -> None:
        super().__init__(parent)
        self.setWindowTitle('SO-101 Control Dashboard')
        self.resize(1400, 860)

        central = QWidget()
        self.setCentralWidget(central)
        outer = QVBoxLayout(central)

        if mock_mode:
            mock_banner = QLabel('MOCK MODE -- no physical robot, no real ROS 2 system connected')
            mock_banner.setObjectName('mockBanner')
            mock_banner.setAlignment(Qt.AlignCenter)
            outer.addWidget(mock_banner)

        tabs = QTabWidget()
        outer.addWidget(tabs)

        control_tab = self._build_control_tab(
            robot_interface, robot_state, torque_manager, joint_limits,
            arm_joint_names, home_position, home_duration_sec, telemetry_update_rate,
        )
        tabs.addTab(control_tab, 'CONTROL')

        joint_control_tab = JointControlWidget(
            joint_control_manager, joint_limits,
            arm_joint_names, gripper_joint_name, telemetry_update_rate,
        )
        tabs.addTab(joint_control_tab, 'JOINT CONTROL')

        telemetry_tab = TelemetryWidget(plot_manager, all_joint_names, telemetry_history_seconds)
        tabs.addTab(telemetry_tab, 'TELEMETRY')

        program_tab = ProgramWidget(program_manager, waypoint_manager, program_directory, waypoint_directory)
        tabs.addTab(program_tab, 'PROGRAMS')

        trajectory_tab = TrajectoryWidget(trajectory_recorder, trajectory_manager, trajectory_directory)
        tabs.addTab(trajectory_tab, 'TRAJECTORIES')

        rosbag_tab = RosbagWidget(
            robot_interface, rosbag_available_topics, rosbag_default_topics, rosbag_directory, mock_mode,
        )
        tabs.addTab(rosbag_tab, 'ROSBAG')

        tabs.addTab(_settings_tab(config_dir), 'SETTINGS')

    def _build_control_tab(
        self,
        robot_interface: RobotInterface,
        robot_state: RobotState,
        torque_manager: TorqueManager,
        joint_limits: JointLimits,
        arm_joint_names: List[str],
        home_position: Dict[str, float],
        home_duration_sec: float,
        telemetry_update_rate: float,
    ) -> QWidget:
        tab = QWidget()
        layout = QHBoxLayout(tab)

        left = QVBoxLayout()
        control_panel = ControlPanel(
            torque_manager, robot_interface, joint_limits,
            arm_joint_names, home_position, home_duration_sec,
        )
        status_widget = StatusWidget(robot_state)
        left.addWidget(control_panel, stretch=2)
        left.addWidget(status_widget, stretch=1)

        left_container = QWidget()
        left_container.setLayout(left)
        left_container.setMaximumWidth(340)

        joint_table = JointTable(robot_state, telemetry_update_rate)

        layout.addWidget(left_container)
        layout.addWidget(joint_table, stretch=1)
        return tab
