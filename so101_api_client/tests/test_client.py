"""Unit tests for the so101_api client. No network: the requests.Session is
replaced with a stub that records calls and returns canned responses."""
from __future__ import annotations

import pytest
import requests

from so101_api import (
    ApiError, ConnectionFailed, NotSupported, ProgramBusy, ProgramNotFound, SO101, SO101Error,
    TargetNotFound,
)


class FakeResponse:
    def __init__(self, status_code=200, json_body=None, text='', content=None):
        self.status_code = status_code
        self._json = json_body
        self.text = text
        if content is None:
            content = b'{}' if json_body is not None else (text.encode() if text else b'')
        self.content = content

    def json(self):
        if self._json is None:
            raise ValueError('no json')
        return self._json


class FakeSession:
    def __init__(self):
        self.calls = []
        self.response = FakeResponse(200, {'ok': True})
        self.raises = None

    def request(self, method, url, **kwargs):
        self.calls.append((method, url, kwargs))
        if self.raises is not None:
            raise self.raises
        return self.response


@pytest.fixture
def robot():
    r = SO101('http://host:8000/')
    s = FakeSession()
    # namespaces share the client's session object, so swap it everywhere
    r.session = r.programs.session = r.targets.session = r.robot.session = r.gripper.session = s
    r.fake = s
    return r


def last(robot):
    return robot.fake.calls[-1]


def test_base_url_trailing_slash_stripped_and_defaults():
    assert SO101('http://h:1/').base_url == 'http://h:1'
    d = SO101()
    assert d.base_url == 'http://127.0.0.1:8000'
    assert d.timeout == 10.0


def test_namespaces_share_client_config():
    r = SO101('http://h:1', timeout=2.5)
    for ns in (r.programs, r.targets, r.robot, r.gripper):
        assert ns.base_url == 'http://h:1'
        assert ns.timeout == 2.5
        assert ns.session is r.session


@pytest.mark.parametrize('call, method, path', [
    (lambda r: r.get_state(), 'GET', '/api/robot/state'),
    (lambda r: r.get_status(), 'GET', '/api/robot/status'),
    (lambda r: r.get_torque_state(), 'GET', '/api/robot/torque'),
    (lambda r: r.enable_torque(), 'POST', '/api/robot/torque/enable'),
    (lambda r: r.disable_torque(), 'POST', '/api/robot/torque/disable'),
    (lambda r: r.get_programs(), 'GET', '/api/programs'),
    (lambda r: r.get_program('P'), 'GET', '/api/programs/P'),
    (lambda r: r.run_program('P'), 'POST', '/api/programs/P/run'),
    (lambda r: r.stop(), 'POST', '/api/program/stop'),
    (lambda r: r.get_targets(), 'GET', '/api/targets'),
    (lambda r: r.get_target('home'), 'GET', '/api/targets/home'),
    (lambda r: r.programs.list(), 'GET', '/api/programs'),
    (lambda r: r.targets.list(), 'GET', '/api/targets'),
    (lambda r: r.robot.state(), 'GET', '/api/robot/state'),
])
def test_endpoint_routing(robot, call, method, path):
    assert call(robot) == {'ok': True}
    m, url, kw = last(robot)
    assert (m, url) == (method, f'http://host:8000{path}')
    assert kw['timeout'] == robot.timeout


def test_move_to_target_defaults_and_overrides(robot):
    robot.move_to_target('home')
    _, url, kw = last(robot)
    assert url.endswith('/api/targets/home/move')
    assert kw['json'] == {
        'duration_sec': 3.0, 'velocity_scaling': 1.0, 'acceleration_scaling': 1.0, 'wait': True,
    }
    robot.move_to_target('home', duration_sec=1.5, velocity_scaling=0.5, wait=False)
    assert last(robot)[2]['json'] == {
        'duration_sec': 1.5, 'velocity_scaling': 0.5, 'acceleration_scaling': 1.0, 'wait': False,
    }


def test_gripper_calls(robot):
    robot.open_gripper()
    assert last(robot)[1].endswith('/api/gripper/open')
    assert last(robot)[2]['json'] == {'wait': True}
    robot.close_gripper(wait=False)
    assert last(robot)[1].endswith('/api/gripper/close')
    assert last(robot)[2]['json'] == {'wait': False}


def test_gripper_position_optional_max_effort(robot):
    robot.gripper.position(0.4)
    assert last(robot)[2]['json'] == {'position': 0.4, 'wait': True}
    robot.gripper.position(0.4, max_effort=2.0, wait=False)
    assert last(robot)[2]['json'] == {'position': 0.4, 'wait': False, 'max_effort': 2.0}


def test_post_without_body_sends_json_none(robot):
    robot.run_program('P')
    assert last(robot)[2]['json'] is None


def test_empty_body_returns_none(robot):
    robot.fake.response = FakeResponse(204, None, content=b'')
    assert robot.stop() is None


def test_list_response_passthrough(robot):
    robot.fake.response = FakeResponse(200, [{'name': 'a'}, {'name': 'b'}])
    assert [p['name'] for p in robot.get_programs()] == ['a', 'b']


@pytest.mark.parametrize('code, exc', [
    ('PROGRAM_NOT_FOUND', ProgramNotFound),
    ('TARGET_NOT_FOUND', TargetNotFound),
    ('PROGRAM_BUSY', ProgramBusy),
    ('NOT_SUPPORTED', NotSupported),
    ('SOMETHING_ELSE', ApiError),
])
def test_error_codes_map_to_exceptions(robot, code, exc):
    robot.fake.response = FakeResponse(409, {'error': code, 'message': 'nope'})
    with pytest.raises(exc) as ei:
        robot.run_program('P')
    assert type(ei.value) is exc
    assert isinstance(ei.value, ApiError) and isinstance(ei.value, SO101Error)
    assert ei.value.status_code == 409
    assert ei.value.error_code == code
    assert ei.value.message == 'nope'
    assert '409' in str(ei.value) and 'nope' in str(ei.value)


def test_error_with_non_json_body(robot):
    robot.fake.response = FakeResponse(500, None, text='Internal Server Error')
    with pytest.raises(ApiError) as ei:
        robot.get_state()
    assert ei.value.status_code == 500
    assert ei.value.error_code == 'UNKNOWN_ERROR'
    assert ei.value.message == 'Internal Server Error'


def test_error_json_missing_fields(robot):
    robot.fake.response = FakeResponse(400, {'detail': 'x'}, text='raw')
    with pytest.raises(ApiError) as ei:
        robot.get_state()
    assert ei.value.error_code == 'UNKNOWN_ERROR'
    assert ei.value.message == 'raw'


def test_status_399_is_success_400_is_error(robot):
    robot.fake.response = FakeResponse(399, {'ok': 1})
    assert robot.get_state() == {'ok': 1}
    robot.fake.response = FakeResponse(400, {'error': 'X', 'message': 'm'})
    with pytest.raises(ApiError):
        robot.get_state()


def test_connection_failure_wrapped(robot):
    original = requests.exceptions.ConnectTimeout('boom')
    robot.fake.raises = original
    with pytest.raises(ConnectionFailed) as ei:
        robot.get_state()
    assert ei.value.base_url == 'http://host:8000'
    assert ei.value.original is original
    assert ei.value.__cause__ is original
    assert 'http://host:8000' in str(ei.value)
    assert not isinstance(ei.value, ApiError)
    assert isinstance(ei.value, SO101Error)


def test_namespace_errors_propagate(robot):
    robot.fake.response = FakeResponse(404, {'error': 'TARGET_NOT_FOUND', 'message': 'no'})
    with pytest.raises(TargetNotFound):
        robot.targets.move('missing')
