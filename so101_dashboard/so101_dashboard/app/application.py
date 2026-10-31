"""Top-level orchestrator: loads config, constructs the ROS bridge (real or
mock), RobotState, TorqueManager, all the Phase 2/4/5/6 managers, and
MainWindow, wiring them together. This is the one place that knows about
all of them; everything else only knows the pieces it directly depends on.
"""
from __future__ import annotations

import os
import signal
import sys
from typing import Optional

from PySide6.QtCore import QTimer
from PySide6.QtWidgets import QApplication

from .autosave import LIBRARY_FILENAME, AutoSaver, restore_missing
from .main_window import MainWindow
from .styles import DARK_STYLESHEET
from ..api.bridge import DashboardBridge
from ..api.context import ApiContext
from ..api.server import ApiServer
from ..plotting.plot_manager import PlotManager
from ..recording.trajectory_recorder import TrajectoryRecorder
from ..robot.gripper_manager import GripperManager
from ..robot.joint_control_manager import JointControlManager
from ..robot.joint_limits import JointLimits
from ..robot.program_manager import ProgramManager
from ..robot.robot_state import RobotState
from ..robot.torque_manager import TorqueManager
from ..robot.trajectory_manager import TrajectoryManager
from ..robot.waypoint_manager import WaypointManager
from ..ros.mock_robot_interface import MockRobotInterface
from ..ros.robot_interface import RobotInterface
from ..utils.config_loader import find_config_dir, load_yaml
from ..utils.logging import get_logger

_log = get_logger('app.application')


def _resolve_storage_dir(path: str, package_root: str) -> str:
    """Storage paths in config.yaml are relative (e.g. "data/waypoints").
    Resolve them against the package root (source checkout, or install
    share dir's parent) rather than whatever the current working directory
    happens to be when the app is launched."""
    if os.path.isabs(path):
        return path
    return os.path.normpath(os.path.join(package_root, path))


class Application:
    def __init__(self, mock_mode: bool = False, config_dir: Optional[str] = None) -> None:
        self.mock_mode = mock_mode
        self._config_dir = find_config_dir(config_dir)
        _log.info(f'Loading config from {self._config_dir}')
        package_root = os.path.dirname(self._config_dir)

        self._config = load_yaml(self._config_dir, 'config.yaml')
        self._robot_config: dict = self._config['robot']
        self._telemetry_config: dict = self._config.get('telemetry', {})
        self._recording_config: dict = self._config.get('recording', {})
        self._motion_config: dict = self._config.get('motion', {})
        self._safety_config: dict = self._config.get('safety', {})
        self._storage_config: dict = self._config.get('storage', {})
        self._rosbag_config: dict = self._config.get('rosbag', {})

        self._joint_limits = JointLimits.from_yaml(
            os.path.join(self._config_dir, 'joint_limits.yaml')
        )
        home_data = load_yaml(self._config_dir, 'home_position.yaml')
        self._home_position: dict = home_data.get('home_position', {})

        self._qt_app = QApplication.instance() or QApplication(sys.argv)
        self._qt_app.setStyleSheet(DARK_STYLESHEET)

        arm_joint_names = list(self._robot_config['joint_names'])
        gripper_joint_name = self._robot_config['gripper_joint_name']
        all_joint_names = arm_joint_names + [gripper_joint_name]

        if mock_mode:
            _log.warning('Running in MOCK MODE -- no real ROS 2 connection, no real torque control')
            self._robot_interface = MockRobotInterface(arm_joint_names, gripper_joint_name)
        else:
            self._robot_interface = RobotInterface(self._robot_config, self._safety_config)

        self._robot_state = RobotState(
            self._robot_interface,
            arm_joint_names,
            gripper_joint_name,
            self._safety_config.get('joint_state_timeout', 0.5),
        )
        self._robot_state.mock_mode = mock_mode

        self._torque_manager = TorqueManager(
            self._robot_interface,
            self._joint_limits,
            arm_joint_names,
            gripper_joint_name,
            self._motion_config,
            self._safety_config,
        )
        self._torque_manager.state_changed.connect(self._robot_state.on_torque_state_changed)

        self._plot_manager = PlotManager(
            self._robot_state,
            all_joint_names,
            self._telemetry_config.get('update_rate', 30.0),
            self._telemetry_config.get('history_seconds', 30.0),
        )

        self._waypoint_manager = WaypointManager(
            self._robot_interface, self._robot_state, self._joint_limits,
            arm_joint_names, gripper_joint_name,
        )

        self._program_manager = ProgramManager(
            self._robot_interface, self._robot_state, self._joint_limits,
            arm_joint_names, gripper_joint_name, self._motion_config,
        )

        self._gripper_manager = GripperManager(
            self._robot_interface, self._joint_limits, gripper_joint_name, self._motion_config,
        )

        self._joint_control_manager = JointControlManager(
            self._robot_interface, self._robot_state, self._joint_limits,
            arm_joint_names, gripper_joint_name, self._home_position,
            self._motion_config.get('home_trajectory_duration', 4.0), self._motion_config,
        )

        self._trajectory_recorder = TrajectoryRecorder(self._robot_interface, all_joint_names)
        self._trajectory_recorder.recording_changed.connect(
            lambda active: self._robot_state.set_trajectory_recording(active)
        )
        self._trajectory_manager = TrajectoryManager(
            self._robot_interface, self._joint_limits, arm_joint_names, gripper_joint_name,
            self._motion_config.get('gripper_hold_max_effort', 50.0),
        )

        waypoint_dir = _resolve_storage_dir(
            self._storage_config.get('waypoint_directory', 'data/waypoints'), package_root
        )
        program_dir = _resolve_storage_dir(
            self._storage_config.get('program_directory', 'data/programs'), package_root
        )
        trajectory_dir = _resolve_storage_dir(
            self._storage_config.get('trajectory_directory', 'data/trajectories'), package_root
        )
        rosbag_dir = _resolve_storage_dir(
            self._storage_config.get('rosbag_directory', 'data/bags'), package_root
        )

        # colcon build wipes install/ (where these relative dirs land for
        # the installed package), so bring back anything that only
        # survives in the backup, then keep both in sync from here on.
        backup_dir = os.path.expanduser(self._storage_config.get('backup_directory', '~/so100_waypoints'))
        restore_missing(backup_dir, waypoint_dir, program_dir)
        library_file = os.path.join(waypoint_dir, LIBRARY_FILENAME)
        if os.path.exists(library_file):
            try:
                self._waypoint_manager.load(library_file)
            except Exception as exc:  # noqa: BLE001
                _log.warning(f'Could not load waypoint library {library_file}: {exc}')

        # Populate the PROGRAMS tab's tree with every program already saved
        # to disk, so multiple programs show up ready to switch between
        # rather than only whichever one is loaded manually.
        self._program_manager.load_all_from_directory(program_dir)

        self._autosaver = AutoSaver(
            self._waypoint_manager, self._program_manager, waypoint_dir, program_dir, backup_dir,
        )

        self._main_window = MainWindow(
            self._robot_interface,
            self._robot_state,
            self._torque_manager,
            self._joint_limits,
            self._plot_manager,
            self._waypoint_manager,
            self._program_manager,
            self._joint_control_manager,
            self._trajectory_recorder,
            self._trajectory_manager,
            arm_joint_names,
            gripper_joint_name,
            all_joint_names,
            self._home_position,
            self._motion_config.get('home_trajectory_duration', 4.0),
            self._telemetry_config.get('update_rate', 30.0),
            self._telemetry_config.get('history_seconds', 30.0),
            waypoint_dir,
            program_dir,
            trajectory_dir,
            rosbag_dir,
            self._rosbag_config.get('available_topics', []),
            self._rosbag_config.get('default_topics', []),
            self._config_dir,
            mock_mode,
        )

        self._qt_app.aboutToQuit.connect(self._on_about_to_quit)

        # -- REST API: an ADDITION on top of everything above, sharing the
        #    exact same manager instances MainWindow uses (see api/context.py)
        #    -- not a second control pathway. See so101_dashboard/api/ for
        #    the FastAPI app; DashboardBridge is what makes it safe for the
        #    API's background thread to call into these Qt-main-thread
        #    manager objects at all.
        self._api_config: dict = self._config.get('api', {})
        self._api_server: Optional[ApiServer] = None
        if self._api_config.get('enabled', True):
            self._api_bridge = DashboardBridge()
            api_ctx = ApiContext(
                bridge=self._api_bridge,
                robot_state=self._robot_state,
                torque_manager=self._torque_manager,
                waypoint_manager=self._waypoint_manager,
                program_manager=self._program_manager,
                gripper_manager=self._gripper_manager,
                joint_control_manager=self._joint_control_manager,
                joint_limits=self._joint_limits,
                arm_joint_names=arm_joint_names,
                gripper_joint_name=gripper_joint_name,
                mock_mode=mock_mode,
            )
            self._api_server = ApiServer(
                api_ctx,
                host=self._api_config.get('host', '127.0.0.1'),
                port=int(self._api_config.get('port', 8000)),
            )

    def run(self) -> int:
        self._robot_interface.start()
        if self._api_server is not None:
            self._api_server.start()
        self._main_window.show()
        self._install_sigint_handler()
        return self._qt_app.exec()

    def _install_sigint_handler(self) -> None:
        """Ctrl+C in the terminal is a SIGINT, but Python's default handler
        for it never gets a chance to run while Qt's C++ event loop has
        control -- so without this, Ctrl+C does nothing (or the process
        just hangs) until you click back into the GUI. A signal handler
        that quits through the normal QApplication.quit() path (so
        aboutToQuit -> _on_about_to_quit() still stops/joins the ROS
        worker thread, same as closing the window normally) plus a
        short-interval QTimer, which is what actually makes Python regain
        control often enough to notice the signal was raised at all --
        Qt's exec() alone never hands back to the interpreter.

        The timer tick also RE-INSTALLS the handler every time, not just
        once: pyqtgraph's Qt compatibility layer is known to reset SIGINT
        back to Python's plain default_int_handler the first time certain
        Qt/GL machinery gets touched (confirmed here -- without this
        re-assert, Ctrl+C degraded into a raw KeyboardInterrupt traceback
        dumped from whatever telemetry-plot redraw callback happened to
        be running, instead of a clean shutdown). Re-asserting every tick
        makes this self-healing regardless of what resets it or when."""
        def _handler(*_args) -> None:
            self._qt_app.quit()

        def _tick() -> None:
            signal.signal(signal.SIGINT, _handler)

        _tick()
        self._sigint_poke_timer = QTimer(self._qt_app)
        self._sigint_poke_timer.timeout.connect(_tick)
        self._sigint_poke_timer.start(200)

    def _on_about_to_quit(self) -> None:
        self._autosaver.flush()
        _log.info('Shutting down...')
        if self._api_server is not None:
            self._api_server.stop()
        self._robot_interface.stop()
        if not self.mock_mode:
            self._robot_interface.wait(2000)
