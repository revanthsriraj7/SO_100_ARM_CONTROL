"""GET /api/programs, GET /api/programs/{name}, POST /api/programs/{name}/run,
POST /api/program/stop, POST /api/program/pause, POST /api/program/resume.

Every handler calls straight into the existing ProgramManager -- the same
registry/executor the PROGRAMS tab's tree and RUN/STOP buttons use. There
is no separate execution engine here: run() dispatches through
ProgramManager exactly as widgets/program_widget.py does, and this module
returns as soon as the run is accepted (RUNNING), not when it finishes --
poll GET /api/robot/state or GET /api/programs/{name} for progress, same
as the spec's BobbinWinding example expects.

pause/resume are reported as 501 NOT_SUPPORTED: ProgramManager has no
pause/resume of a running program (only TrajectoryManager does, for raw
trajectory playback -- a different feature). Inventing one here would
violate "do not invent an unsafe implementation."
"""
from __future__ import annotations

from typing import List, Optional

from fastapi import APIRouter, Depends

from ..context import ApiContext
from ..deps import get_ctx
from ..errors import InvalidRequestError, NotSupportedError, ProgramBusyError, ProgramNotFoundError, TargetNotFoundError
from ..models import (
    AddProgramElementRequest, CreateProgramRequest, EditResponse,
    ProgramDetail, ProgramElementResponse, ProgramSummary, RunProgramResponse, StopProgramResponse,
)
from ...robot.program_manager import ElementType, ExecutionState

router = APIRouter(tags=['programs'])


@router.get('/api/programs', response_model=List[ProgramSummary], summary='List known programs')
def list_programs(ctx: ApiContext = Depends(get_ctx)) -> List[ProgramSummary]:
    def _read() -> List[ProgramSummary]:
        pm = ctx.program_manager
        result = []
        for name in pm.program_names:
            program = pm.get_program(name)
            result.append(ProgramSummary(
                name=name, num_elements=len(program.elements),
                loop_count=program.loop_count, is_active=(name == pm.active_name),
            ))
        return result
    return ctx.bridge.call_on_gui_thread(_read)


def _element_response(pm, index: int, el) -> ProgramElementResponse:
    return ProgramElementResponse(
        index=index, type=el.element_type.value, name=el.display_name,
        state=pm.element_state(el).name,
        wait_duration_sec=el.wait_duration_sec if el.element_type == ElementType.WAIT else None,
        velocity_scaling=el.velocity_scaling, acceleration_scaling=el.acceleration_scaling,
    )


@router.get('/api/programs/{program_name}', response_model=ProgramDetail, summary='Get one program\'s ordered elements')
def get_program(program_name: str, ctx: ApiContext = Depends(get_ctx)) -> ProgramDetail:
    def _read() -> ProgramDetail:
        pm = ctx.program_manager
        program = pm.get_program(program_name)
        if program is None:
            raise ProgramNotFoundError(program_name)
        is_active = program_name == pm.active_name
        elements = [_element_response(pm, i, el) for i, el in enumerate(program.elements)]
        return ProgramDetail(
            name=program.name, elements=elements, loop_count=program.loop_count,
            execution_state=pm.execution_state.name if is_active else ExecutionState.IDLE.name,
            current_element_index=(pm.current_index if is_active and pm.is_running() and pm.current_index >= 0 else None),
            current_loop=pm.current_loop if is_active else None,
        )
    return ctx.bridge.call_on_gui_thread(_read)


@router.post(
    '/api/programs/{program_name}/run', response_model=RunProgramResponse,
    summary='Run a program',
    description='Starts the program via the existing executor and returns immediately once it is accepted (RUNNING) -- does not block for the full run. Returns 409 if a program is already running.',
)
def run_program(program_name: str, ctx: ApiContext = Depends(get_ctx)) -> RunProgramResponse:
    def _do() -> RunProgramResponse:
        pm = ctx.program_manager
        if pm.is_running():
            raise ProgramBusyError()
        if program_name not in pm.program_names:
            raise ProgramNotFoundError(program_name)
        if program_name != pm.active_name:
            if not pm.select_program(program_name):
                raise ProgramBusyError('Could not select the program to run.')
        pm.run()
        state = pm.execution_state
        if state != ExecutionState.RUNNING:
            raise InvalidRequestError(
                f"Program '{program_name}' did not start (state={state.name}). It may be empty."
            )
        return RunProgramResponse(accepted=True, program=program_name, execution_state=state.name, message='Program started.')
    return ctx.bridge.call_on_gui_thread(_do)


@router.post(
    '/api/program/stop', response_model=StopProgramResponse,
    summary='Safely stop the running program',
    description='Uses ProgramManager.stop() -- cancels the in-flight goal via the normal cancel-all mechanism and marks the program STOPPED. Never kills any process/node directly.',
)
def stop_program(ctx: ApiContext = Depends(get_ctx)) -> StopProgramResponse:
    def _do() -> StopProgramResponse:
        pm = ctx.program_manager
        pm.stop()
        return StopProgramResponse(execution_state=pm.execution_state.name, message='Stop requested.')
    return ctx.bridge.call_on_gui_thread(_do)


def _select_for_edit(ctx: ApiContext, program_name: str):
    pm = ctx.program_manager
    if pm.is_running():
        raise ProgramBusyError('Cannot edit programs while one is running.')
    if program_name not in pm.program_names:
        raise ProgramNotFoundError(program_name)
    pm.select_program(program_name)
    return pm


@router.post('/api/programs', response_model=EditResponse, summary='Create a new empty program',
             description='The new program becomes the active one. Autosaved to disk once it has elements.')
def create_program(req: CreateProgramRequest, ctx: ApiContext = Depends(get_ctx)) -> EditResponse:
    def _do() -> EditResponse:
        pm = ctx.program_manager
        if not pm.new_program(req.name):
            raise ProgramBusyError('Cannot create a program while one is running.')
        return EditResponse(message=f"Created program '{pm.active_name}'.")
    return ctx.bridge.call_on_gui_thread(_do)


@router.post(
    '/api/programs/{program_name}/elements', response_model=EditResponse, summary='Add a step to a program',
    description="Appends (or inserts at `index`) a step: type 'target' (move to a saved target), 'record_current_pose', 'jaw_open', 'jaw_close' or 'wait'.",
)
def add_program_element(program_name: str, req: AddProgramElementRequest, ctx: ApiContext = Depends(get_ctx)) -> EditResponse:
    def _do() -> EditResponse:
        pm = _select_for_edit(ctx, program_name)
        before = len(pm.program.elements)
        if req.type == 'target':
            match = next((wp for wp in ctx.waypoint_manager.waypoints if wp.name == req.target_name), None)
            if match is None:
                raise TargetNotFoundError(str(req.target_name))
            pm.add_existing_waypoint(match, req.index)
        elif req.type == 'record_current_pose':
            if not pm.add_waypoint(req.index):
                raise InvalidRequestError('Cannot record the current pose (no joint data yet).')
        elif req.type == 'jaw_open':
            pm.add_jaw_open(req.index)
        elif req.type == 'jaw_close':
            pm.add_jaw_close(req.index)
        elif req.type == 'wait':
            pm.add_wait(req.index, req.duration_sec)
        else:
            raise InvalidRequestError(f"Unknown element type '{req.type}'.")
        if len(pm.program.elements) == before:
            raise InvalidRequestError('Step was not added.')
        return EditResponse(message=f"Added {req.type} to '{program_name}' ({before + 1} steps).")
    return ctx.bridge.call_on_gui_thread(_do)


@router.delete('/api/programs/{program_name}/elements/{index}', response_model=EditResponse, summary='Delete a step from a program')
def delete_program_element(program_name: str, index: int, ctx: ApiContext = Depends(get_ctx)) -> EditResponse:
    def _do() -> EditResponse:
        pm = _select_for_edit(ctx, program_name)
        if not (0 <= index < len(pm.program.elements)):
            raise InvalidRequestError(f'No step at index {index}.')
        pm.delete_element(index)
        return EditResponse(message=f"Deleted step {index} of '{program_name}'.")
    return ctx.bridge.call_on_gui_thread(_do)


@router.post('/api/program/pause', summary='Not supported', description='ProgramManager has no pause of a running program -- always returns 501.')
def pause_program() -> None:
    raise NotSupportedError('Pausing a running program is not supported by the current executor.')


@router.post('/api/program/resume', summary='Not supported', description='ProgramManager has no resume of a running program -- always returns 501.')
def resume_program() -> None:
    raise NotSupportedError('Resuming a running program is not supported by the current executor.')
