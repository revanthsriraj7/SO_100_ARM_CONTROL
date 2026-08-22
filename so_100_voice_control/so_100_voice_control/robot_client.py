"""Thin HTTP client for the so101_dashboard REST API.

This is the ONLY way the voice package touches the robot. It never
imports so101_dashboard, never reads program YAML/JSON files, and never
opens a second control pathway -- every method here is a straight call to
an endpoint that already exists (see so101_dashboard/api/routes/*.py).
The one addition made on the dashboard side for this integration was
POST /api/robot/home, which itself just calls the existing
JointControlManager.go_home() -- everything else below was already
there.
"""
from __future__ import annotations

from typing import Any, Dict, List, Optional

import requests

from . import config
from .utils import get_logger

_log = get_logger('robot_client')


class ApiClientError(RuntimeError):
    """Raised for any non-2xx response or connection failure. Carries the
    API's own {error, message} body when available so callers (and TTS)
    can surface something meaningful instead of a raw exception."""

    def __init__(self, message: str, status_code: Optional[int] = None, error_code: Optional[str] = None) -> None:
        super().__init__(message)
        self.message = message
        self.status_code = status_code
        self.error_code = error_code


class SO101Client:
    """Client for the dashboard's public REST API. One instance is cheap
    to create (it just wraps a requests.Session) and safe to reuse across
    an entire voice-control session."""

    def __init__(self, base_url: Optional[str] = None, timeout: Optional[float] = None) -> None:
        self.base_url = (base_url or config.API_URL).rstrip('/')
        self.timeout = config.API_TIMEOUT_SEC if timeout is None else timeout
        self._session = requests.Session()

    # -- transport --

    def _request(self, method: str, path: str, **kwargs) -> Any:
        url = f'{self.base_url}{path}'
        try:
            resp = self._session.request(method, url, timeout=self.timeout, **kwargs)
        except requests.exceptions.RequestException as exc:
            raise ApiClientError(f'Could not reach dashboard API at {url}: {exc}') from exc

        if not resp.ok:
            error_code, message = None, resp.text
            try:
                body = resp.json()
                error_code = body.get('error')
                message = body.get('message', message)
            except ValueError:
                pass
            raise ApiClientError(message, status_code=resp.status_code, error_code=error_code)

        if resp.status_code == 204 or not resp.content:
            return None
        return resp.json()

    # -- health --

    def is_reachable(self) -> bool:
        """Best-effort connectivity check -- used at startup before the
        node starts listening (Phase 7, step 2)."""
        try:
            self._request('GET', '/api/robot/status')
            return True
        except ApiClientError:
            return False

    # -- robot state --

    def get_state(self) -> Dict[str, Any]:
        return self._request('GET', '/api/robot/state')

    def get_status(self) -> Dict[str, Any]:
        return self._request('GET', '/api/robot/status')

    # -- programs --

    def get_programs(self) -> List[Dict[str, Any]]:
        return self._request('GET', '/api/programs')

    def get_program(self, name: str) -> Dict[str, Any]:
        return self._request('GET', f'/api/programs/{name}')

    def run_program(self, name: str) -> Dict[str, Any]:
        return self._request('POST', f'/api/programs/{name}/run')

    def stop_program(self) -> Dict[str, Any]:
        return self._request('POST', '/api/program/stop')

    # -- gripper --

    def open_gripper(self, wait: bool = True) -> Dict[str, Any]:
        return self._request('POST', '/api/gripper/open', json={'wait': wait})

    def close_gripper(self, wait: bool = True) -> Dict[str, Any]:
        return self._request('POST', '/api/gripper/close', json={'wait': wait})

    # -- home --

    def go_home(self) -> Dict[str, Any]:
        return self._request('POST', '/api/robot/home')
