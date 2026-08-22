"""Unit tests for DashboardBridge -- the cross-thread call mechanism the
API layer uses to reach Qt-main-thread manager objects. Uses the real
qapp fixture (a real QApplication event loop) and a real background
thread, since this mechanism's whole job is to be safe across an actual
thread boundary -- a fake/synchronous stand-in would not exercise it.
"""
from __future__ import annotations

import threading

from PySide6.QtCore import QObject, Signal
from PySide6.QtTest import QTest

from so101_dashboard.api.bridge import DashboardBridge, GuiCallTimeout


class _SignalHost(QObject):
    finished = Signal(object)


def _pump_until(predicate, timeout_ms=3000, step_ms=10) -> bool:
    waited = 0
    while not predicate() and waited < timeout_ms:
        QTest.qWait(step_ms)
        waited += step_ms
    return predicate()


def test_call_on_gui_thread_from_background_thread(qapp):
    bridge = DashboardBridge()
    result = {}

    def worker():
        result['value'] = bridge.call_on_gui_thread(lambda: 21 * 2)

    t = threading.Thread(target=worker)
    t.start()
    assert _pump_until(lambda: not t.is_alive())
    t.join(timeout=2)
    assert result['value'] == 42


def test_call_on_gui_thread_reraises_exception_on_caller_thread(qapp):
    bridge = DashboardBridge()
    result = {}

    def boom():
        raise ValueError('kaboom')

    def worker():
        try:
            bridge.call_on_gui_thread(boom)
        except ValueError as exc:
            result['error'] = str(exc)

    t = threading.Thread(target=worker)
    t.start()
    assert _pump_until(lambda: not t.is_alive())
    t.join(timeout=2)
    assert result['error'] == 'kaboom'


def test_call_on_gui_thread_inline_when_already_on_gui_thread(qapp):
    # Calling from the GUI thread itself (as a plain function call, no
    # background thread involved) must run inline, not deadlock.
    bridge = DashboardBridge()
    assert bridge.call_on_gui_thread(lambda: 'inline') == 'inline'


def test_call_on_gui_thread_timeout(qapp):
    bridge = DashboardBridge()
    result = {}

    def worker():
        try:
            bridge.call_on_gui_thread(lambda: 1, timeout=0.05)
        except GuiCallTimeout as exc:
            result['timed_out'] = True

    # Starve the event loop deliberately by not pumping it at all before
    # the timeout elapses.
    t = threading.Thread(target=worker)
    t.start()
    t.join(timeout=2)
    assert result.get('timed_out') is True


def test_call_and_await_signal(qapp):
    bridge = DashboardBridge()
    host = _SignalHost()
    dispatched = []
    result = {}

    def dispatch():
        dispatched.append('go')
        return 'request-123'

    def worker():
        result['value'] = bridge.call_and_await_signal(dispatch, host.finished, timeout=5.0)

    t = threading.Thread(target=worker)
    t.start()
    # Wait for the background thread's dispatch to actually run on the GUI
    # thread (proves the connect-then-dispatch happens atomically before
    # any emission could be missed).
    assert _pump_until(lambda: bool(dispatched))
    assert dispatched == ['go']

    # Simulate the async robot response arriving (as RobotInterface's
    # trajectory_result -> GripperManager.goal_finished would, on the GUI
    # thread) -- this must unblock the waiting background thread.
    host.finished.emit('outcome-payload')
    assert _pump_until(lambda: not t.is_alive())
    t.join(timeout=2)

    dispatch_result, payload = result['value']
    assert dispatch_result == 'request-123'
    assert payload == 'outcome-payload'
