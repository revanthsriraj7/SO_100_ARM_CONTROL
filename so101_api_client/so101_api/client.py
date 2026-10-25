"""Thin Python client for the SO-101 dashboard's REST API.

Deliberately NOT a robot-control implementation of its own: every method
here is a single HTTP call to an endpoint documented at
http://<host>/docs on the running dashboard. All actual robot control,
program execution, torque handling, and safety logic lives in the
dashboard process (so101_dashboard) -- this package only knows how to
talk HTTP to it.

    from so101_api import SO101
    robot = SO101("http://127.0.0.1:8000")
    robot.run_program("BobbinWinding")
    while True:
        state = robot.get_state()
        if state["program"]["state"] in ("FINISHED", "STOPPED", "FAILED"):
            break
"""
from __future__ import annotations

from typing import Any, Dict, List, Optional

import requests

from .exceptions import ConnectionFailed, build_api_error

_DEFAULT_TIMEOUT = 10.0


class _HttpMixin:
    base_url: str
    timeout: float
    session: requests.Session

    def _request(self, method: str, path: str, **kwargs) -> Any:
        url = f'{self.base_url}{path}'
        kwargs.setdefault('timeout', self.timeout)
        try:
            resp = self.session.request(method, url, **kwargs)
        except requests.exceptions.RequestException as exc:
            raise ConnectionFailed(self.base_url, exc) from exc

        if resp.status_code >= 400:
            try:
                body = resp.json()
                error_code = body.get('error', 'UNKNOWN_ERROR')
                message = body.get('message', resp.text)
            except ValueError:
                error_code = 'UNKNOWN_ERROR'
                message = resp.text
            raise build_api_error(resp.status_code, error_code, message)

        if not resp.content:
            return None
        return resp.json()

    def _get(self, path: str, **kwargs) -> Any:
        return self._request('GET', path, **kwargs)

    def _post(self, path: str, json: Optional[dict] = None, **kwargs) -> Any:
        return self._request('POST', path, json=json, **kwargs)


class _ProgramsNamespace(_HttpMixin):
    """RoboDK-style nested access: robot.programs.list() / .get() / .run()."""

    def __init__(self, client: 'SO101') -> None:
        self.base_url = client.base_url
        self.timeout = client.timeout
        self.session = client.session

    def list(self) -> List[Dict[str, Any]]:
        return self._get('/api/programs')

    def get(self, name: str) -> Dict[str, Any]:
        return self._get(f'/api/programs/{name}')

    def run(self, name: str) -> Dict[str, Any]:
        return self._post(f'/api/programs/{name}/run')

    def stop(self) -> Dict[str, Any]:
        return self._post('/api/program/stop')


class _TargetsNamespace(_HttpMixin):
    def __init__(self, client: 'SO101') -> None:
        self.base_url = client.base_url
        self.timeout = client.timeout
        self.session = client.session

    def list(self) -> List[Dict[str, Any]]:
        return self._get('/api/targets')

    def get(self, name: str) -> Dict[str, Any]:
        return self._get(f'/api/targets/{name}')

    def move(
        self, name: str, duration_sec: float = 3.0,
        velocity_scaling: float = 1.0, acceleration_scaling: float = 1.0, wait: bool = True,
    ) -> Dict[str, Any]:
        return self._post(
            f'/api/targets/{name}/move',
            json={
                'duration_sec': duration_sec, 'velocity_scaling': velocity_scaling,
                'acceleration_scaling': acceleration_scaling, 'wait': wait,
            },
        )


class _RobotNamespace(_HttpMixin):
    def __init__(self, client: 'SO101') -> None:
        self.base_url = client.base_url
        self.timeout = client.timeout
        self.session = client.session

    def state(self) -> Dict[str, Any]:
        return self._get('/api/robot/state')

    def status(self) -> Dict[str, Any]:
        return self._get('/api/robot/status')

    def torque(self) -> Dict[str, Any]:
        return self._get('/api/robot/torque')

    def enable_torque(self) -> Dict[str, Any]:
        return self._post('/api/robot/torque/enable')

    def disable_torque(self) -> Dict[str, Any]:
        return self._post('/api/robot/torque/disable')


class _GripperNamespace(_HttpMixin):
    def __init__(self, client: 'SO101') -> None:
        self.base_url = client.base_url
        self.timeout = client.timeout
        self.session = client.session

    def open(self, wait: bool = True) -> Dict[str, Any]:
        return self._post('/api/gripper/open', json={'wait': wait})

    def close(self, wait: bool = True) -> Dict[str, Any]:
        return self._post('/api/gripper/close', json={'wait': wait})

    def position(self, position: float, max_effort: Optional[float] = None, wait: bool = True) -> Dict[str, Any]:
        body = {'position': position, 'wait': wait}
        if max_effort is not None:
            body['max_effort'] = max_effort
        return self._post('/api/gripper/position', json=body)


class SO101(_HttpMixin):
    """Connects to a running so101_dashboard's REST API.

        robot = SO101("http://127.0.0.1:8000")

    Offers both a flat convenience surface (get_state(), run_program(...),
    ...) and RoboDK-style nested namespaces (robot.programs, robot.targets,
    robot.robot, robot.gripper) over the exact same HTTP calls -- pick
    whichever reads better for your script.
    """

    def __init__(self, base_url: str = 'http://127.0.0.1:8000', timeout: float = _DEFAULT_TIMEOUT) -> None:
        self.base_url = base_url.rstrip('/')
        self.timeout = timeout
        self.session = requests.Session()

        self.programs = _ProgramsNamespace(self)
        self.targets = _TargetsNamespace(self)
        self.robot = _RobotNamespace(self)
        self.gripper = _GripperNamespace(self)

    # -- flat convenience surface (spec section 7) --

    def get_state(self) -> Dict[str, Any]:
        return self.robot.state()

    def get_status(self) -> Dict[str, Any]:
        return self.robot.status()

    def get_programs(self) -> List[Dict[str, Any]]:
        return self.programs.list()

    def get_program(self, name: str) -> Dict[str, Any]:
        return self.programs.get(name)

    def run_program(self, name: str) -> Dict[str, Any]:
        return self.programs.run(name)

    def stop(self) -> Dict[str, Any]:
        return self.programs.stop()

    def get_torque_state(self) -> Dict[str, Any]:
        return self.robot.torque()

    def enable_torque(self) -> Dict[str, Any]:
        return self.robot.enable_torque()

    def disable_torque(self) -> Dict[str, Any]:
        return self.robot.disable_torque()

    def open_gripper(self, wait: bool = True) -> Dict[str, Any]:
        return self.gripper.open(wait=wait)

    def close_gripper(self, wait: bool = True) -> Dict[str, Any]:
        return self.gripper.close(wait=wait)

    def get_target(self, name: str) -> Dict[str, Any]:
        return self.targets.get(name)

    def get_targets(self) -> List[Dict[str, Any]]:
        return self.targets.list()

    def move_to_target(self, name: str, **kwargs) -> Dict[str, Any]:
        return self.targets.move(name, **kwargs)
