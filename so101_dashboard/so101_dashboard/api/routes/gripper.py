"""POST /api/gripper/open, /close, /position -- all via GripperManager,
which itself just wraps RobotInterface.request_gripper_goal() with the
same config-driven open/closed positions the rest of the app uses. No
raw servo commands are ever reachable through this API.
"""
from __future__ import annotations

from fastapi import APIRouter, Depends

from ..context import ApiContext
from ..deps import get_ctx
from ..errors import InvalidRequestError
from ..models import GripperCommandRequest, GripperPositionRequest, GripperStateResponse

router = APIRouter(prefix='/api/gripper', tags=['gripper'])


def _respond(ctx: ApiContext, dispatch, wait: bool, target_position: float) -> GripperStateResponse:
    if not wait:
        ctx.bridge.call_on_gui_thread(dispatch)
        return GripperStateResponse(position=target_position, accepted=True, succeeded=None, message='Gripper goal requested (not waited for).')
    _, outcome = ctx.bridge.call_and_await_signal(dispatch, ctx.gripper_manager.goal_finished, timeout=10.0)
    return GripperStateResponse(position=target_position, accepted=outcome.accepted, succeeded=outcome.succeeded, message=outcome.message)


@router.post('/open', response_model=GripperStateResponse, summary='Open the gripper')
def open_gripper(body: GripperCommandRequest = GripperCommandRequest(), ctx: ApiContext = Depends(get_ctx)) -> GripperStateResponse:
    gm = ctx.gripper_manager
    return _respond(ctx, gm.open, body.wait, gm.open_position)


@router.post('/close', response_model=GripperStateResponse, summary='Close the gripper')
def close_gripper(body: GripperCommandRequest = GripperCommandRequest(), ctx: ApiContext = Depends(get_ctx)) -> GripperStateResponse:
    gm = ctx.gripper_manager
    return _respond(ctx, gm.close, body.wait, gm.closed_position)


@router.post('/position', response_model=GripperStateResponse, summary='Move the gripper to an arbitrary position')
def set_gripper_position(body: GripperPositionRequest, ctx: ApiContext = Depends(get_ctx)) -> GripperStateResponse:
    gm = ctx.gripper_manager

    def dispatch():
        try:
            return gm.move_to(body.position, body.max_effort)
        except ValueError as exc:
            # GripperManager.move_to() validates against JointLimits and
            # raises ValueError -- translated to a proper 400 here rather
            # than falling through to the generic 500 handler.
            raise InvalidRequestError(str(exc)) from exc

    return _respond(ctx, dispatch, body.wait, body.position)
