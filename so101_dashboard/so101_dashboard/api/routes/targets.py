"""GET /api/targets, GET /api/targets/{name}, POST /api/targets/{name}/move.

"Targets" are WaypointManager's existing named waypoint library -- the
same list the WAYPOINTS tree in the dashboard shows and edits. No second
target representation is introduced here; this module only reads that
list and, for /move, calls WaypointManager.move_to() (the same safe
pathway the dashboard's own "Move To" button uses).
"""
from __future__ import annotations

from typing import List, Optional

from fastapi import APIRouter, Depends

from ..context import ApiContext
from ..deps import get_ctx
from ..errors import InvalidRequestError, TargetNotFoundError
from ..models import MoveResultResponse, MoveToTargetRequest, TargetDetail, TargetSummary
from ...robot import kinematics

router = APIRouter(prefix='/api/targets', tags=['targets'])


def _find_index(ctx: ApiContext, name: str) -> int:
    for i, wp in enumerate(ctx.waypoint_manager.waypoints):
        if wp.name == name:
            return i
    raise TargetNotFoundError(name)


@router.get('', response_model=List[TargetSummary], summary='List saved targets')
def list_targets(ctx: ApiContext = Depends(get_ctx)) -> List[TargetSummary]:
    def _read() -> List[TargetSummary]:
        return [TargetSummary(name=wp.name, joint_names=list(wp.joint_names)) for wp in ctx.waypoint_manager.waypoints]
    return ctx.bridge.call_on_gui_thread(_read)


@router.get('/{target_name}', response_model=TargetDetail, summary='Get one target')
def get_target(target_name: str, ctx: ApiContext = Depends(get_ctx)) -> TargetDetail:
    def _read() -> TargetDetail:
        idx = _find_index(ctx, target_name)
        wp = ctx.waypoint_manager.waypoints[idx]
        cartesian: Optional[dict] = None
        by_name = dict(zip(wp.joint_names, wp.joint_positions))
        arm_positions = [by_name[n] for n in ctx.arm_joint_names if n in by_name]
        if len(arm_positions) == len(ctx.arm_joint_names):
            try:
                x, y, z = kinematics.forward(arm_positions)
                cartesian = {'x': x, 'y': y, 'z': z}
            except Exception:  # noqa: BLE001 -- FK is best-effort here, never fatal to the response
                cartesian = None
        return TargetDetail(
            name=wp.name, joint_names=list(wp.joint_names),
            joints=dict(zip(wp.joint_names, wp.joint_positions)),
            cartesian_pose=cartesian,
        )
    return ctx.bridge.call_on_gui_thread(_read)


@router.post('/{target_name}/move', response_model=MoveResultResponse, summary='Move the robot to a saved target')
def move_to_target(
    target_name: str, body: MoveToTargetRequest, ctx: ApiContext = Depends(get_ctx)
) -> MoveResultResponse:
    def _dispatch() -> None:
        idx = _find_index(ctx, target_name)
        wp = ctx.waypoint_manager.waypoints[idx]
        # Validated up front so an invalid target fails fast with a 400
        # instead of silently never firing move_finished and timing out --
        # WaypointManager.move_to() itself just logs+returns on this case.
        errors = ctx.joint_limits.validate_positions(wp.joint_names, wp.joint_positions)
        if errors:
            raise InvalidRequestError('Cannot move to target:\n' + '\n'.join(errors))
        ctx.waypoint_manager.move_to(
            idx, body.duration_sec,
            velocity_scaling=body.velocity_scaling, acceleration_scaling=body.acceleration_scaling,
        )

    if not body.wait:
        ctx.bridge.call_on_gui_thread(_dispatch)
        return MoveResultResponse(accepted=True, succeeded=None, message='Move requested (not waited for).')

    _, outcome = ctx.bridge.call_and_await_signal(_dispatch, ctx.waypoint_manager.move_finished, timeout=30.0)
    return MoveResultResponse(accepted=outcome.accepted, succeeded=outcome.succeeded, message=outcome.message)
