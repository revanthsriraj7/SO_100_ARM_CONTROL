"""End-to-end test of the standalone so101_api Python client SDK against a
REAL (loopback, random-port) uvicorn instance of the FastAPI app -- not
just the in-process TestClient used in test_api.py. This is what proves
the SDK's actual HTTP layer (so101_api/client.py) works, not just the
FastAPI route functions.

Deliberately a single server lifecycle covering several assertions
(rather than one server per test) -- spinning up/tearing down many real
uvicorn instances back-to-back in one pytest process was observed to be
flaky in this environment; one lifecycle is enough to prove the SDK's
HTTP layer works end to end, and the per-route logic itself is already
covered without any real networking in test_api.py.

Skipped automatically if the so101_api package (so101_api_client/,
installed separately -- see its README) isn't on the path, since it's a
standalone package outside this ROS package's own dependencies.
"""
from __future__ import annotations

import socket
import threading
import time

import pytest
import uvicorn
from PySide6.QtTest import QTest

so101_api = pytest.importorskip('so101_api')

from so101_dashboard.api.server import create_app  # noqa: E402
from so101_dashboard.storage.waypoint_storage import Waypoint  # noqa: E402

from test_api import ALL_JOINTS, build_ctx  # noqa: E402


def _pumped(fn, timeout_ms=10000):
    """Runs fn() on a background thread while pumping the qapp event loop
    on the calling (main/GUI) thread. Needed here for the same reason as
    in test_api.py: DashboardBridge needs the GUI thread's Qt event loop
    actually running to deliver a queued call, and the SO101 client's
    HTTP calls block whichever thread makes them."""
    result = {}

    def worker() -> None:
        try:
            result['value'] = fn()
        except Exception as exc:  # noqa: BLE001
            result['exception'] = exc

    t = threading.Thread(target=worker)
    t.start()
    waited = 0
    while t.is_alive() and waited < timeout_ms:
        QTest.qWait(10)
        waited += 10
    t.join(timeout=2)
    if 'exception' in result:
        raise result['exception']
    if 'value' not in result:
        raise RuntimeError('timed out waiting for the SO101 client call to complete')
    return result['value']


def _free_port() -> int:
    s = socket.socket()
    s.bind(('127.0.0.1', 0))
    port = s.getsockname()[1]
    s.close()
    return port


@pytest.fixture
def live_api(fake_ri):
    ctx = build_ctx(fake_ri)
    ctx.waypoint_manager.add_existing(Waypoint.create('home', ALL_JOINTS, [0.0, 0.0, 0.5]))
    ctx.program_manager.add_existing_waypoint(Waypoint.create('wp1', ALL_JOINTS, [0.0, 0.0, 0.5]))
    ctx.program_manager.rename_program('Demo')

    app = create_app(ctx)
    port = _free_port()
    config = uvicorn.Config(app, host='127.0.0.1', port=port, log_level='error')
    server = uvicorn.Server(config)
    server.install_signal_handlers = False
    thread = threading.Thread(target=server.run, daemon=True)
    thread.start()
    for _ in range(100):
        if getattr(server, 'started', False):
            break
        time.sleep(0.05)
    try:
        yield f'http://127.0.0.1:{port}', ctx
    finally:
        server.should_exit = True
        thread.join(timeout=5.0)


def test_client_end_to_end(live_api):
    """One live server, several assertions: state/programs/targets reads,
    run -> duplicate-run rejection -> stop, a 404, and the nested
    (RoboDK-style) namespace surface -- covering the SDK's HTTP layer in
    a single server lifecycle."""
    base_url, _ctx = live_api
    robot = so101_api.SO101(base_url)

    state = _pumped(robot.get_state)
    assert state['connected'] is True

    programs = _pumped(robot.get_programs)
    assert any(p['name'] == 'Demo' for p in programs)

    targets = _pumped(robot.get_targets)
    assert any(t['name'] == 'home' for t in targets)
    target = _pumped(lambda: robot.get_target('home'))
    assert target['joints'] == {'j1': 0.0, 'j2': 0.0, 'gripper': 0.5}

    result = _pumped(lambda: robot.run_program('Demo'))
    assert result['execution_state'] == 'RUNNING'

    with pytest.raises(so101_api.ProgramBusy):
        _pumped(lambda: robot.run_program('Demo'))

    stop_result = _pumped(robot.stop)
    assert stop_result['execution_state'] == 'STOPPED'

    with pytest.raises(so101_api.ProgramNotFound):
        _pumped(lambda: robot.get_program('DoesNotExist'))

    # Nested (RoboDK-style) namespace surface -- same underlying calls.
    gripper_result = _pumped(lambda: robot.gripper.open(wait=False))
    assert gripper_result['accepted'] is True
    torque = _pumped(robot.robot.torque)
    assert torque['state'] == 'UNKNOWN'
