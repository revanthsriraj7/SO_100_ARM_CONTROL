"""Exceptions raised by the SO101 client. Mirror the REST API's error
codes 1:1 -- see so101_dashboard/api/errors.py on the server side."""
from __future__ import annotations

from typing import Optional


class SO101Error(Exception):
    """Base class for all errors raised by this client."""


class ApiError(SO101Error):
    """The server returned a structured {"error": ..., "message": ...}
    response with a non-2xx status code."""

    def __init__(self, status_code: int, error_code: str, message: str) -> None:
        super().__init__(f'[{status_code} {error_code}] {message}')
        self.status_code = status_code
        self.error_code = error_code
        self.message = message


class ProgramNotFound(ApiError):
    pass


class TargetNotFound(ApiError):
    pass


class ProgramBusy(ApiError):
    pass


class NotSupported(ApiError):
    pass


_CODE_TO_EXCEPTION = {
    'PROGRAM_NOT_FOUND': ProgramNotFound,
    'TARGET_NOT_FOUND': TargetNotFound,
    'PROGRAM_BUSY': ProgramBusy,
    'NOT_SUPPORTED': NotSupported,
}


def build_api_error(status_code: int, error_code: str, message: str) -> ApiError:
    cls = _CODE_TO_EXCEPTION.get(error_code, ApiError)
    return cls(status_code, error_code, message)


class ConnectionFailed(SO101Error):
    """Could not reach the dashboard's API server at all (wrong URL, not
    running, network error, etc.) -- distinct from ApiError, which means
    the server WAS reached and responded with an error."""

    def __init__(self, base_url: str, original: Optional[Exception] = None) -> None:
        super().__init__(f'Could not reach the SO-101 API at {base_url}: {original}')
        self.base_url = base_url
        self.original = original
