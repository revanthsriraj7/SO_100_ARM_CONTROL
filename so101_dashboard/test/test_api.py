"""Unit tests for the REST API (so101_dashboard/api/).

Most tests call the route functions directly (e.g. robot.get_robot_state(ctx))
against real managers (ProgramManager, WaypointManager, TorqueManager,
GripperManager, RobotState) driven by FakeRobotInterface -- the project's
existing no-hardware-needed pattern (see conftest.py). This is a true
in-process unit test of each route's logic: DashboardBridge's
call_on_gui_thread() runs inline (no threading) when called from the same
thread that owns the QApplication, which is exactly what happens here,
so there's nothing async/networked to synchronize -- no different, in
principle, from calling any other manager method directly in a test.

A single end-to-end test (test_live_server_smoke) drives the same
in-process app through a real HTTP round trip (fastapi.testclient) to
prove the ASGI wiring, JSON (de)serialization, and exception-handler
status-code mapping actually work -- kept to one server lifecycle,
since spinning up/tearing down many uvicorn/TestClient instances in a
single pytest process was observed to be flaky (crashes) in this
environment when repeated many times back-to-back; the per-route logic
itself is already fully covered by the direct-call tests above it.

DashboardBridge's actual cross-thread behavior (the part that only
matters when a real background thread is involved, as in production) has
its own dedicated test: test_api_bridge.py.
"""
from __future__ import annotations

import threading

import pytest
from fastapi.testclient import TestClient
from PySide6.QtTest import QTest

from so101_dashboard.api.bridge import DashboardBridge
from so101_dashboard.api.context import ApiContext
from so101_dashboard.api.errors import (
    ConflictError, InvalidRequestError, NotSupportedError, ProgramBusyError, ProgramNotFoundError,
    TargetNotFoundError,
)
from so101_dashboard.api.models import (
    GripperCommandRequest, GripperPositionRequest, MoveToTargetRequest,
)
from so101_dashboard.api.routes import gripper as gripper_routes
from so101_dashboard.api.routes import programs as program_routes
from so101_dashboard.api.routes import robot as robot_routes
from so101_dashboard.api.routes import targets as target_routes
from so101_dashboard.api.server import create_app
from so101_dashboard.robot.gripper_manager import GripperManager
from so101_dashboard.robot.joint_limits import JointLimit, JointLimits
from so101_dashboard.robot.program_manager import ProgramManager
from so101_dashboard.robot.robot_state import RobotState
from so101_dashboard.robot.torque_manager import TorqueManager
from so101_dashboard.robot.waypoint_manager import WaypointManager
from so101_dashboard.storage.waypoint_storage import Waypoint
from so101_dashboard.utils.signals import JointStateSnapshot, ServiceCallOutcome

ARM_JOINTS = ['j1', 'j2']
GRIPPER = 'gripper'
ALL_JOINTS = ARM_JOINTS + [GRIPPER]

MOTION_CONFIG = {
    'gripper_open_position': 1.5,
    'gripper_closed_position': 0.0,
    'gripper_hold_max_effort': 50.0,
    'program_waypoint_duration': 2.0,
    'hold_trajectory_duration': 0.5,
}
SAFETY_CONFIG = {'joint_state_timeout': 5.0, 'action_goal_timeout': 1.0}


def make_limits():
    return JointLimits({
        'j1': JointLimit(min_position=-3.14, max_position=3.14),
        'j2': JointLimit(min_position=-3.14, max_position=3.14),
        'gripper': JointLimit(min_position=-0.2, max_position=1.6),
    })


def build_ctx(fake_ri, feed_joint_state=True):
    limits = make_limits()
    robot_state = RobotState(fake_ri, ARM_JOINTS, GRIPPER, SAFETY_CONFIG['joint_state_timeout'])
    torque_manager = TorqueManager(fake_ri, limits, ARM_JOINTS, GRIPPER, MOTION_CONFIG, SAFETY_CONFIG)
    waypoint_manager = WaypointManager(fake_ri, robot_state, limits, ARM_JOINTS, GRIPPER)
    program_manager = ProgramManager(fake_ri, robot_state, limits, ARM_JOINTS, GRIPPER, MOTION_CONFIG)
    gripper_manager = GripperManager(fake_ri, limits, GRIPPER, MOTION_CONFIG)

    if feed_joint_state:
        fake_ri.joint_state_received.emit(JointStateSnapshot(
            stamp_sec=0.0,
            positions={'j1': 0.1, 'j2': 0.2, 'gripper': 0.3},
            velocities={n: 0.0 for n in ALL_JOINTS},
            efforts={n: 0.0 for n in ALL_JOINTS},
        ))

    return ApiContext(
        bridge=DashboardBridge(),
        robot_state=robot_state,
        torque_manager=torque_manager,
        waypoint_manager=waypoint_manager,
        program_manager=program_manager,
        gripper_manager=gripper_manager,
        joint_limits=limits,
        arm_joint_names=ARM_JOINTS,
        gripper_joint_name=GRIPPER,
        mock_mode=True,
    )


def _succeed_last_toggle(fake_ri) -> None:
    request_id = fake_ri.toggle_torque_calls[-1]
    fake_ri.torque_result.emit(ServiceCallOutcome(success=True, message='ok', request_id=request_id))


# -- robot state / status / torque --

def test_get_robot_state(fake_ri):
    ctx = build_ctx(fake_ri)
    resp = robot_routes.get_robot_state(ctx)
    assert resp.connected is True
    assert resp.joints == {'j1': 0.1, 'j2': 0.2}
    assert resp.gripper == {'gripper': 0.3}
    assert resp.program.state == 'IDLE'


def test_get_robot_status(fake_ri):
    ctx = build_ctx(fake_ri)
    resp = robot_routes.get_robot_status(ctx)
    assert resp.mock_mode is True
    assert resp.ros_connected is True


def test_get_torque_state(fake_ri):
    ctx = build_ctx(fake_ri)
    resp = robot_routes.get_torque_state(ctx)
    assert resp.state == 'UNKNOWN'
    assert resp.can_disable is True   # UNKNOWN allows FREE MOVE


def test_torque_enable_disable(fake_ri):
    ctx = build_ctx(fake_ri)
    resp = robot_routes.disable_torque(ctx)
    assert resp.state == 'TOGGLING'
    _succeed_last_toggle(fake_ri)
    assert robot_routes.get_torque_state(ctx).state == 'FREE_MOVE'

    resp = robot_routes.enable_torque(ctx)
    assert resp.accepted is True

    # From CAPTURING_POSITION, a second enable attempt is rejected -- 409.
    with pytest.raises(ConflictError):
        robot_routes.enable_torque(ctx)


# -- targets --

def test_list_and_get_target(fake_ri):
    ctx = build_ctx(fake_ri)
    ctx.waypoint_manager.add_existing(Waypoint.create('home', ALL_JOINTS, [0.0, 0.0, 0.5]))

    targets = target_routes.list_targets(ctx)
    assert [t.name for t in targets] == ['home']

    detail = target_routes.get_target('home', ctx)
    assert detail.joints == {'j1': 0.0, 'j2': 0.0, 'gripper': 0.5}


def test_get_target_not_found(fake_ri):
    ctx = build_ctx(fake_ri)
    with pytest.raises(TargetNotFoundError):
        target_routes.get_target('nope', ctx)


def test_move_to_target_no_wait(fake_ri):
    ctx = build_ctx(fake_ri)
    ctx.waypoint_manager.add_existing(Waypoint.create('home', ALL_JOINTS, [0.0, 0.0, 0.5]))
    resp = target_routes.move_to_target('home', MoveToTargetRequest(wait=False), ctx)
    assert resp.accepted is True
    assert len(fake_ri.arm_goal_calls) == 1


def test_move_to_unknown_target_404(fake_ri):
    ctx = build_ctx(fake_ri)
    with pytest.raises(TargetNotFoundError):
        target_routes.move_to_target('nope', MoveToTargetRequest(wait=False), ctx)


# -- programs --

def test_list_and_get_program(fake_ri):
    ctx = build_ctx(fake_ri)
    ctx.program_manager.add_existing_waypoint(Waypoint.create('wp1', ALL_JOINTS, [0.0, 0.0, 0.5]))
    ctx.program_manager.add_jaw_close()
    ctx.program_manager.rename_program('MyProgram')

    summaries = {p.name: p for p in program_routes.list_programs(ctx)}
    assert 'MyProgram' in summaries
    assert summaries['MyProgram'].num_elements == 2
    assert summaries['MyProgram'].is_active is True

    detail = program_routes.get_program('MyProgram', ctx)
    assert [e.type for e in detail.elements] == ['waypoint', 'jaw_close']
    assert detail.execution_state == 'IDLE'


def test_get_program_not_found(fake_ri):
    ctx = build_ctx(fake_ri)
    with pytest.raises(ProgramNotFoundError):
        program_routes.get_program('NoSuchProgram', ctx)


def test_run_stop_and_duplicate_run(fake_ri):
    ctx = build_ctx(fake_ri)
    ctx.program_manager.add_existing_waypoint(Waypoint.create('wp1', ALL_JOINTS, [0.0, 0.0, 0.5]))
    ctx.program_manager.rename_program('RunMe')

    resp = program_routes.run_program('RunMe', ctx)
    assert resp.accepted is True
    assert resp.execution_state == 'RUNNING'

    # Still running (FakeRobotInterface never auto-responds to the arm
    # goal) -- a second run must be rejected, not silently start again.
    with pytest.raises(ProgramBusyError):
        program_routes.run_program('RunMe', ctx)

    state = robot_routes.get_robot_state(ctx)
    assert state.program.state == 'RUNNING'
    assert state.program.current_element_index == 0

    resp = program_routes.stop_program(ctx)
    assert resp.execution_state == 'STOPPED'
    assert fake_ri.cancel_all_calls == 1


def test_run_unknown_program_404(fake_ri):
    ctx = build_ctx(fake_ri)
    with pytest.raises(ProgramNotFoundError):
        program_routes.run_program('NoSuchProgram', ctx)


def test_run_empty_program_400(fake_ri):
    ctx = build_ctx(fake_ri)
    ctx.program_manager.rename_program('Empty')
    with pytest.raises(InvalidRequestError):
        program_routes.run_program('Empty', ctx)


def test_pause_resume_not_supported(fake_ri):
    with pytest.raises(NotSupportedError):
        program_routes.pause_program()
    with pytest.raises(NotSupportedError):
        program_routes.resume_program()


# -- gripper --

def test_gripper_open_close_no_wait(fake_ri):
    ctx = build_ctx(fake_ri)
    resp = gripper_routes.open_gripper(GripperCommandRequest(wait=False), ctx)
    assert resp.position == 1.5
    resp = gripper_routes.close_gripper(GripperCommandRequest(wait=False), ctx)
    assert resp.position == 0.0
    assert len(fake_ri.gripper_goal_calls) == 2


def test_gripper_position_out_of_limits(fake_ri):
    ctx = build_ctx(fake_ri)
    with pytest.raises(InvalidRequestError):
        gripper_routes.set_gripper_position(GripperPositionRequest(position=99.0, wait=False), ctx)


# -- API/dashboard state consistency: the whole point of the design --

def test_api_and_dashboard_share_the_same_manager_state(fake_ri):
    """Proves there is exactly one ProgramManager/WaypointManager instance
    -- editing 'through the dashboard' (i.e. calling the manager directly,
    as widgets do) is immediately visible via the API route functions, and
    vice versa."""
    ctx = build_ctx(fake_ri)

    ctx.waypoint_manager.add_existing(Waypoint.create('shared', ALL_JOINTS, [0.1, 0.1, 0.1]))
    assert any(t.name == 'shared' for t in target_routes.list_targets(ctx))

    ctx.program_manager.rename_program('SharedProgram')
    ctx.program_manager.add_jaw_open()
    assert ctx.program_manager.program.elements[-1].element_type.value == 'jaw_open'
    detail = program_routes.get_program('SharedProgram', ctx)
    assert len(detail.elements) == 1


# -- one true end-to-end HTTP round trip, proving the ASGI wiring itself --

def _pumped_request(client: TestClient, method: str, path: str, timeout_ms=10000, **kwargs):
    """TestClient executes route code on a thread other than the caller's;
    DashboardBridge needs the GUI thread's Qt event loop actually pumped
    to deliver its queued call, so run the request on a worker thread
    while this (GUI/qapp) thread pumps events -- mirrors production
    (uvicorn's thread + QApplication.exec())."""
    result = {}

    def worker() -> None:
        result['response'] = getattr(client, method)(path, **kwargs)

    t = threading.Thread(target=worker)
    t.start()
    waited = 0
    while t.is_alive() and waited < timeout_ms:
        QTest.qWait(10)
        waited += 10
    t.join(timeout=2)
    if 'response' not in result:
        raise RuntimeError(f'{method} {path} timed out waiting for the GUI-thread bridge call')
    return result['response']


def test_live_server_smoke(fake_ri):
    ctx = build_ctx(fake_ri)
    ctx.waypoint_manager.add_existing(Waypoint.create('home', ALL_JOINTS, [0.0, 0.0, 0.5]))
    ctx.program_manager.rename_program('Empty')

    client = TestClient(create_app(ctx))

    resp = _pumped_request(client, 'get', '/api/robot/state')
    assert resp.status_code == 200
    assert resp.json()['connected'] is True

    resp = _pumped_request(client, 'get', '/api/targets/home')
    assert resp.status_code == 200

    resp = _pumped_request(client, 'get', '/api/targets/nope')
    assert resp.status_code == 404
    assert resp.json() == {'error': 'TARGET_NOT_FOUND', 'message': "Target 'nope' does not exist."}

    resp = _pumped_request(client, 'post', '/api/programs/Empty/run')
    assert resp.status_code == 400
    assert resp.json()['error'] == 'INVALID_REQUEST'

    resp = _pumped_request(client, 'post', '/api/program/pause')
    assert resp.status_code == 501

    resp = _pumped_request(client, 'get', '/docs')
    assert resp.status_code == 200
