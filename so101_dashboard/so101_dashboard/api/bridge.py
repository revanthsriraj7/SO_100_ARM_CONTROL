"""Cross-thread call bridge: lets the FastAPI/uvicorn server -- which runs
in its own background thread with its own asyncio loop -- safely call into
the existing Qt-main-thread manager objects (ProgramManager,
WaypointManager, TorqueManager, GripperManager, RobotState), which are
QObjects and were never designed to be touched from any thread but the
one running QApplication.exec().

This is the ONLY mechanism the API layer uses to reach the managers.
There is deliberately no second, API-owned copy of any state and no
alternate path to the robot -- every route handler ends up calling a
method that already exists on one of the real managers, via this bridge.

Mechanism: a Qt Signal(object) carrying an arbitrary zero-arg callable.
Signals are safe to *emit* from any thread; Qt auto-queues delivery to a
QueuedConnection slot running on the receiver's (here: the GUI/main)
thread. call_on_gui_thread() emits the signal, then blocks the calling
(API) thread on a threading.Event until the GUI thread has actually run
the callable and stashed its result (or exception) -- so callers get a
normal synchronous return value or a re-raised exception, without needing
to know anything about Qt.

(QMetaObject.invokeMethod(..., Qt.BlockingQueuedConnection, ...) was
considered instead, but returning an arbitrary Python object -- a dict, a
dataclass -- through it is awkward; a plain callable + Event is simpler
and keeps every route handler's code looking like ordinary Python.)
"""
from __future__ import annotations

import threading
from typing import Any, Callable, TypeVar

from PySide6.QtCore import QObject, Signal

from ..utils.logging import get_logger

_log = get_logger('api.bridge')

T = TypeVar('T')

_DEFAULT_TIMEOUT_SEC = 5.0


class GuiCallTimeout(RuntimeError):
    """The GUI thread never ran the requested callable within the timeout
    -- e.g. the event loop is blocked or the app is shutting down."""


class DashboardBridge(QObject):
    _call_requested = Signal(object)   # carries a zero-arg callable: () -> None

    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self._call_requested.connect(self._run_on_gui_thread)

    def call_on_gui_thread(self, fn: Callable[[], T], timeout: float = _DEFAULT_TIMEOUT_SEC) -> T:
        """Run fn() on the Qt GUI thread and return its result, blocking the
        calling thread until it's done. Re-raises whatever exception fn()
        raised, on the calling thread, so FastAPI's normal exception
        handling (see errors.py) works unmodified. Safe to call from the
        GUI thread itself too (e.g. in tests) -- runs fn() inline rather
        than deadlocking on its own event loop."""
        from PySide6.QtCore import QThread
        from PySide6.QtWidgets import QApplication

        app = QApplication.instance()
        if app is not None and QThread.currentThread() is app.thread():
            return fn()

        done = threading.Event()
        box: dict = {}

        def _wrapped() -> None:
            try:
                box['result'] = fn()
            except Exception as exc:  # noqa: BLE001 -- deliberately re-raised on the caller's thread
                box['exception'] = exc
            finally:
                done.set()

        self._call_requested.emit(_wrapped)
        if not done.wait(timeout):
            raise GuiCallTimeout(f'GUI thread did not respond within {timeout}s')
        if 'exception' in box:
            raise box['exception']
        return box.get('result')

    def _run_on_gui_thread(self, fn: Callable[[], None]) -> None:
        # Always executes on the GUI thread -- this slot is only ever
        # entered via the queued connection from _call_requested.
        fn()

    def call_and_await_signal(
        self,
        dispatch: Callable[[], T],
        signal: Any,
        timeout: float = 10.0,
    ) -> tuple:
        """For actions that are dispatched (a goal sent) but only complete
        asynchronously once the robot/ROS side reports back via a Qt
        signal -- e.g. GripperManager.goal_finished, WaypointManager.
        move_finished. Connects to `signal` and calls dispatch() atomically
        on the GUI thread (so nothing can complete and fire the signal
        before the connection exists), then blocks the calling thread until
        the signal fires (or times out).

        The connected callback is a plain function, not a QObject slot, so
        Qt delivers it via a direct call on whichever thread emits the
        signal (the GUI thread) -- it just needs to be safe to call from
        there, which setting a threading.Event is.

        Returns (dispatch_result, signal_payload).
        """
        done = threading.Event()
        box: dict = {}

        def _on_signal(payload=None) -> None:
            if 'signal_payload' in box:
                return  # already satisfied by an earlier emission -- ignore extras
            box['signal_payload'] = payload
            done.set()

        def _setup() -> None:
            signal.connect(_on_signal)
            try:
                box['dispatch_result'] = dispatch()
            except Exception as exc:  # noqa: BLE001
                signal.disconnect(_on_signal)
                box['dispatch_exception'] = exc
                done.set()

        self.call_on_gui_thread(_setup)
        if 'dispatch_exception' in box:
            raise box['dispatch_exception']

        ok = done.wait(timeout)

        def _teardown() -> None:
            try:
                signal.disconnect(_on_signal)
            except (TypeError, RuntimeError):
                pass  # already disconnected, or never connected -- fine

        self.call_on_gui_thread(_teardown)

        if not ok:
            raise GuiCallTimeout(f'Timed out after {timeout}s waiting for the robot to respond')
        return box.get('dispatch_result'), box.get('signal_payload')
