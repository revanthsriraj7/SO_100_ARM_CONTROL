"""Typed API errors -> clean {"error": CODE, "message": ...} JSON with the
correct HTTP status code. Route handlers (and the managers they call
into) raise these; register_exception_handlers() wires them -- plus a
catch-all for anything unexpected -- into the FastAPI app so no internal
stack trace or exception repr ever reaches a client.
"""
from __future__ import annotations

from fastapi import FastAPI, Request
from fastapi.responses import JSONResponse

from ..utils.logging import get_logger

_log = get_logger('api.errors')


class ApiError(Exception):
    status_code: int = 500
    code: str = 'INTERNAL_ERROR'

    def __init__(self, message: str) -> None:
        super().__init__(message)
        self.message = message


class NotFoundError(ApiError):
    status_code = 404
    code = 'NOT_FOUND'


class ProgramNotFoundError(NotFoundError):
    code = 'PROGRAM_NOT_FOUND'

    def __init__(self, name: str) -> None:
        super().__init__(f"Program '{name}' does not exist.")


class TargetNotFoundError(NotFoundError):
    code = 'TARGET_NOT_FOUND'

    def __init__(self, name: str) -> None:
        super().__init__(f"Target '{name}' does not exist.")


class ConflictError(ApiError):
    status_code = 409
    code = 'CONFLICT'


class ProgramBusyError(ConflictError):
    code = 'PROGRAM_BUSY'

    def __init__(self, message: str = 'A program is already running.') -> None:
        super().__init__(message)


class InvalidRequestError(ApiError):
    status_code = 400
    code = 'INVALID_REQUEST'


class NotSupportedError(ApiError):
    """The requested operation has no safe underlying implementation in
    this app yet (e.g. program pause/resume) -- reported honestly as 501,
    never faked."""
    status_code = 501
    code = 'NOT_SUPPORTED'


def register_exception_handlers(app: FastAPI) -> None:
    @app.exception_handler(ApiError)
    async def _handle_api_error(_request: Request, exc: ApiError) -> JSONResponse:
        return JSONResponse(status_code=exc.status_code, content={'error': exc.code, 'message': exc.message})

    @app.exception_handler(Exception)
    async def _handle_unexpected(_request: Request, exc: Exception) -> JSONResponse:
        _log.exception(f'Unhandled exception in API request: {exc}')
        return JSONResponse(
            status_code=500,
            content={'error': 'INTERNAL_ERROR', 'message': 'An unexpected internal error occurred.'},
        )
