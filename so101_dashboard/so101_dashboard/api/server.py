"""Builds the FastAPI app and runs it in a background thread (uvicorn owns
its own asyncio loop there) alongside Qt's QApplication.exec() on the main
thread. Application.run() starts/stops this; nothing here ever touches a
manager directly except through the DashboardBridge stashed in
app.state.ctx (see context.py, deps.py).
"""
from __future__ import annotations

import threading
from typing import Optional

from fastapi import FastAPI

from .context import ApiContext
from .errors import register_exception_handlers
from .routes import gripper, programs, robot, targets
from ..utils.logging import get_logger

_log = get_logger('api.server')

API_DESCRIPTION = """
High-level automation interface for the SO-101 dashboard.

This sits **above** the existing robot-control logic -- every endpoint
calls into the same manager objects (ProgramManager, WaypointManager,
TorqueManager, GripperManager, RobotState) that the PySide6 dashboard UI
itself uses, via a thread-safe bridge. There is exactly one control
pathway to the robot; this API does not open a second one, and never
exposes raw servo commands.
"""


def create_app(ctx: ApiContext) -> FastAPI:
    app = FastAPI(
        title='SO-101 Robot Control API',
        description=API_DESCRIPTION,
        version='1.0.0',
    )
    app.state.ctx = ctx
    register_exception_handlers(app)
    app.include_router(robot.router)
    app.include_router(targets.router)
    app.include_router(programs.router)
    app.include_router(gripper.router)
    return app


class ApiServer:
    """Owns the background uvicorn thread. Started/stopped from
    application.py's existing run()/_on_about_to_quit() lifecycle."""

    def __init__(self, ctx: ApiContext, host: str = '127.0.0.1', port: int = 8000) -> None:
        self._app = create_app(ctx)
        self._host = host
        self._port = port
        self._server: Optional[object] = None
        self._thread: Optional[threading.Thread] = None

    def start(self) -> None:
        import uvicorn

        config = uvicorn.Config(self._app, host=self._host, port=self._port, log_level='warning')
        self._server = uvicorn.Server(config)
        # uvicorn.Server installs its own SIGINT/SIGTERM handlers by
        # default, which would fight the dashboard's own signal handling
        # (see Application._install_sigint_handler) -- disabled since this
        # thread isn't the one that should own process-level signals.
        self._server.install_signal_handlers = False

        def _run() -> None:
            _log.info(f'Starting API server on http://{self._host}:{self._port} (docs at /docs)')
            self._server.run()

        self._thread = threading.Thread(target=_run, name='so101-api-server', daemon=True)
        self._thread.start()

    def stop(self) -> None:
        if self._server is not None:
            self._server.should_exit = True
        if self._thread is not None:
            self._thread.join(timeout=5.0)
