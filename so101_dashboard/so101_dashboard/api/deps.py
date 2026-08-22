"""FastAPI dependency: pulls the ApiContext stashed on app.state by
server.py's create_app(), so every route function can just declare
`ctx: ApiContext = Depends(get_ctx)` instead of reaching into globals."""
from __future__ import annotations

from fastapi import Request

from .context import ApiContext


def get_ctx(request: Request) -> ApiContext:
    return request.app.state.ctx
