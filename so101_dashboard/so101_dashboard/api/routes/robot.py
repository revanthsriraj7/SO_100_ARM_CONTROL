"""GET /api/robot/state, /status, /torque, torque enable/disable, and HOME.

Every handler's body is "read/call the real managers via the bridge, then
shape the result into a Pydantic model" -- no state is computed or cached
here. Torque enable/disable reuse TorqueManager.hold_current_position()/
enter_free_move() exactly as the ROBOT CONTROL panel's HOLD/FREE MOVE
buttons do (see widgets/control_panel.py) -- this is not a second torque
mechanism. POST /home is the same story for JointControlManager.go_home()
-- the exact method the MOVE HOME button calls (see widgets/control_panel.py)
-- there was previously no API route for it at all, so this adds one
instead of duplicating its logic.
"""
from __future__ import annotations

import threading

from fastapi import APIRouter, Depends

from ..bridge import GuiCallTimeout
from ..context import ApiContext
from ..deps import get_ctx
from ..errors import ConflictError
from ..models import (
    MoveResultResponse, ProgramStateInfo, RobotStateResponse, RobotStatusResponse,
    TorqueActionResponse, TorqueStateResponse,
)

router = APIRouter(prefix='/api/robot', tags=['robot'])


def _program_state_info(ctx: ApiContext) -> ProgramStateInfo:
    pm = ctx.program_manager
    program = pm.program
    idx = pm.current_index
    element_name = None
    # current_index can land one-past-the-end once a run finishes (see
    # ProgramManager._advance()) -- only meaningful while actually RUNNING.
    is_running = pm.execution_state.name == 'RUNNING'
    if is_running and 0 <= idx < len(program.elements):
        element_name = program.elements[idx].display_name
    else:
        idx = -1
    return ProgramStateInfo(
        name=program.name,
        state=pm.execution_state.name,
        current_element_index=idx if idx >= 0 else None,
        current_element_name=element_name,
        current_loop=pm.current_loop,
        loop_count=program.loop_count,
    )


@router.get(
    '/state', response_model=RobotStateResponse,
    summary='Full robot + program snapshot',
    description='Actual joint feedback from /joint_states (not commanded values), gripper position, and the active program\'s execution state.',
)
def get_robot_state(ctx: ApiContext = Depends(get_ctx)) -> RobotStateResponse:
    def _read() -> RobotStateResponse:
        rs = ctx.robot_state
        connected = rs.is_receiving_joint_states() and not rs.is_joint_state_stale()
        joints = {name: rs.positions[name] for name in ctx.arm_joint_names if name in rs.positions}
        gripper = {ctx.gripper_joint_name: rs.positions[ctx.gripper_joint_name]} if ctx.gripper_joint_name in rs.positions else {}
        return RobotStateResponse(
            connected=connected,
            torque_state=rs.torque_state.name,
            joints=joints,
            gripper=gripper,
            program=_program_state_info(ctx),
        )
    return ctx.bridge.call_on_gui_thread(_read)


@router.get(
    '/status', response_model=RobotStatusResponse,
    summary='Connection / availability status',
)
def get_robot_status(ctx: ApiContext = Depends(get_ctx)) -> RobotStatusResponse:
    def _read() -> RobotStatusResponse:
        rs = ctx.robot_state
        return RobotStatusResponse(
            ros_connected=rs.is_receiving_joint_states(),
            joint_states_receiving=rs.is_receiving_joint_states(),
            joint_states_stale=rs.is_joint_state_stale(),
            controller_active=rs.is_receiving_joint_states() and not rs.is_joint_state_stale(),
            mock_mode=ctx.mock_mode,
        )
    return ctx.bridge.call_on_gui_thread(_read)


@router.get(
    '/torque', response_model=TorqueStateResponse,
    summary='Current torque state',
)
def get_torque_state(ctx: ApiContext = Depends(get_ctx)) -> TorqueStateResponse:
    def _read() -> TorqueStateResponse:
        tm = ctx.torque_manager
        return TorqueStateResponse(
            state=tm.state.name,
            can_enable=tm.can_hold(),
            can_disable=tm.can_enter_free_move(),
            gripper_lock_enabled=tm.gripper_lock_enabled,
        )
    return ctx.bridge.call_on_gui_thread(_read)


@router.post(
    '/torque/enable', response_model=TorqueActionResponse,
    summary='Enable torque (holds the current position)',
    description="Equivalent to the dashboard's HOLD CURRENT POSITION button: captures a fresh, validated joint-state snapshot and commands it as the hold target. Returns 409 if not currently allowed (see can_enable on GET /api/robot/torque).",
)
def enable_torque(ctx: ApiContext = Depends(get_ctx)) -> TorqueActionResponse:
    def _do() -> TorqueActionResponse:
        tm = ctx.torque_manager
        if not tm.can_hold():
            raise ConflictError(f'Cannot enable torque from state {tm.state.name}.')
        tm.hold_current_position()
        return TorqueActionResponse(accepted=True, state=tm.state.name, message='Hold requested.')
    return ctx.bridge.call_on_gui_thread(_do)


@router.post(
    '/torque/disable', response_model=TorqueActionResponse,
    summary='Disable torque (free move)',
    description="Equivalent to the dashboard's FREE MOVE button. Returns 409 if not currently allowed.",
)
def disable_torque(ctx: ApiContext = Depends(get_ctx)) -> TorqueActionResponse:
    def _do() -> TorqueActionResponse:
        tm = ctx.torque_manager
        if not tm.can_enter_free_move():
            raise ConflictError(f'Cannot disable torque from state {tm.state.name}.')
        tm.enter_free_move()
        return TorqueActionResponse(accepted=True, state=tm.state.name, message='Free move requested.')
    return ctx.bridge.call_on_gui_thread(_do)


@router.post(
    '/home', response_model=MoveResultResponse,
    summary='Move the arm to the configured home position',
    description="Equivalent to the dashboard's MOVE HOME button: calls JointControlManager.go_home() "
                'with the same home_position.yaml / home_trajectory_duration config, and waits for the '
                'coordinated arm goal to finish via the same apply_result signal the button uses.',
)
def go_home(ctx: ApiContext = Depends(get_ctx)) -> MoveResultResponse:
    """Not routed through DashboardBridge.call_and_await_signal() like
    move_to_target/gripper are, because go_home() can reject synchronously
    via error_occurred (e.g. an invalid home_position.yaml) instead of
    raising -- that helper only knows how to wait on ONE success signal, so
    a rejection would silently hang it for the full timeout. Waiting on
    both signals here keeps the failure path fast without changing the
    shared helper's return shape for its other callers."""
    jcm = ctx.joint_control_manager
    done = threading.Event()
    box: dict = {}

    def _on_result(outcome) -> None:
        if 'outcome' not in box and 'error' not in box:
            box['outcome'] = outcome
            done.set()

    def _on_error(message: str) -> None:
        if 'outcome' not in box and 'error' not in box:
            box['error'] = message
            done.set()

    def _setup() -> None:
        jcm.apply_result.connect(_on_result)
        jcm.error_occurred.connect(_on_error)
        jcm.go_home()

    ctx.bridge.call_on_gui_thread(_setup)
    ok = done.wait(30.0)

    def _teardown() -> None:
        for signal, slot in ((jcm.apply_result, _on_result), (jcm.error_occurred, _on_error)):
            try:
                signal.disconnect(slot)
            except (TypeError, RuntimeError):
                pass  # already disconnected -- fine

    ctx.bridge.call_on_gui_thread(_teardown)

    if not ok:
        raise GuiCallTimeout('Timed out after 30.0s waiting for the robot to respond')
    if 'error' in box:
        raise ConflictError(box['error'])
    outcome = box['outcome']
    return MoveResultResponse(accepted=outcome.accepted, succeeded=outcome.succeeded, message=outcome.message)
