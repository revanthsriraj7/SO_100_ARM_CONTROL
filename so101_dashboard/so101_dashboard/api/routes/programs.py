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
from ..errors import InvalidRequestError, NotSupportedError, ProgramBusyError, ProgramNotFoundError
from ..models import (
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


@router.post('/api/program/pause', summary='Not supported', description='ProgramManager has no pause of a running program -- always returns 501.')
def pause_program() -> None:
    raise NotSupportedError('Pausing a running program is not supported by the current executor.')


@router.post('/api/program/resume', summary='Not supported', description='ProgramManager has no resume of a running program -- always returns 501.')
def resume_program() -> None:
    raise NotSupportedError('Resuming a running program is not supported by the current executor.')
