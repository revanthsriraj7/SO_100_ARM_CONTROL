"""Free-form instruction -> robot actions, by handing a local Llama (Ollama
tool calling) the dashboard's whole REST API as tools.

There is no command list: every endpoint in the dashboard's /openapi.json
becomes one tool (name, description and parameters taken from the spec),
so a new endpoint is usable the moment the dashboard serves it. The model
reads state, decides which calls to make and in what order, sees each
result, and ends with a short spoken reply.

Guardrails live outside the model: operations matching
config.LLM_BLOCKED_OPS are refused, dry-run only sends reads, the step
count is capped, and the dashboard still validates joint limits and
refuses edits while a program runs -- the model never sees raw servo
commands.
"""
from __future__ import annotations

import json
import re
from collections import deque
from dataclasses import dataclass, field
from typing import Any, Deque, Dict, List, Optional, Tuple
from urllib.parse import quote

import requests

from . import config
from .robot_client import ApiClientError, SO101Client
from .utils import get_logger

_log = get_logger('llm_agent')

_RUN_RE = re.compile(r'^/api/programs/([^/]+)/run$')
_MAX_RESULT_CHARS = 1500
_MOVE_TIMEOUT_SEC = 40.0  # moves with wait=true block until the motion ends

_SYSTEM = """You control an SO-101 robot arm by calling tools. Each tool is one call to the arm's API.
- Look things up before acting: list targets/programs/state to get exact names. Never invent a name.
- Do exactly what the user asked, nothing more. Chain several calls when the request needs it.
- Programs run in the background; starting one returns immediately.
- If a call fails or is refused, say so plainly; do not retry the same call blindly.
- When you are done, reply with ONE short sentence to be spoken aloud (no lists, no JSON).
If the request is not about the robot, or you cannot do it, say so in one sentence."""


@dataclass
class Tool:
    name: str
    method: str
    path: str
    schema: Dict[str, Any]
    path_params: List[str]
    query_params: List[str]
    body_params: List[str]


@dataclass
class AgentResult:
    reply: str
    calls: List[Tuple[str, str]] = field(default_factory=list)  # (METHOD, path) that were sent


def _resolve(schema: Any, components: Dict[str, Any]) -> Any:
    """Inline $refs so the tool schema is self-contained."""
    if isinstance(schema, dict):
        if '$ref' in schema:
            name = schema['$ref'].rsplit('/', 1)[-1]
            return _resolve(components.get(name, {}), components)
        return {k: _resolve(v, components) for k, v in schema.items() if k != 'title'}
    if isinstance(schema, list):
        return [_resolve(v, components) for v in schema]
    return schema


def _flatten_optional(prop: Dict[str, Any]) -> Dict[str, Any]:
    """Pydantic renders Optional[float] as anyOf [float, null]; small
    models handle the plain type better."""
    options = prop.get('anyOf')
    if options:
        non_null = [o for o in options if o.get('type') != 'null']
        if len(non_null) == 1:
            merged = {**{k: v for k, v in prop.items() if k != 'anyOf'}, **non_null[0]}
            return merged
    return prop


def build_tools(openapi: Dict[str, Any]) -> Dict[str, Tool]:
    components = openapi.get('components', {}).get('schemas', {})
    tools: Dict[str, Tool] = {}
    for path, methods in openapi.get('paths', {}).items():
        for method, op in methods.items():
            if method.lower() not in ('get', 'post', 'put', 'patch', 'delete'):
                continue
            if op.get('summary') == 'Not supported':
                continue
            name = re.sub(r'[^a-zA-Z0-9]+', '_', f'{method}{path}').strip('_').lower()
            props: Dict[str, Any] = {}
            required: List[str] = []
            path_params, query_params, body_params = [], [], []
            for param in op.get('parameters', []):
                prop = _flatten_optional(_resolve(param.get('schema', {}), components))
                if param.get('description'):
                    prop['description'] = param['description']
                props[param['name']] = prop
                if param.get('required'):
                    required.append(param['name'])
                (path_params if param['in'] == 'path' else query_params).append(param['name'])
            body = (op.get('requestBody', {}).get('content', {}).get('application/json', {}).get('schema'))
            if body:
                body = _resolve(body, components)
                for key, prop in body.get('properties', {}).items():
                    props[key] = _flatten_optional(prop)
                    body_params.append(key)
                required += body.get('required', [])
            # Summary only: the long descriptions bloat the prompt, and on a
            # CPU-run 3B model prompt size is most of the latency.
            description = op.get('summary') or ''
            schema = {
                'type': 'function',
                'function': {
                    'name': name,
                    'description': f'{method.upper()} {path} -- {description}'.strip(),
                    'parameters': {'type': 'object', 'properties': props, 'required': required},
                },
            }
            tools[name] = Tool(name, method.upper(), path, schema, path_params, query_params, body_params)
    return tools


def _blocked_rules() -> List[Tuple[str, str]]:
    rules = []
    for item in config.LLM_BLOCKED_OPS.split(','):
        parts = item.strip().split(None, 1)
        if len(parts) == 2:
            rules.append((parts[0].upper(), parts[1]))
    return rules


class LlmAgent:
    def __init__(self, client: SO101Client, dry_run: bool = False) -> None:
        self._client = client
        self._dry_run = dry_run
        self._tools: Dict[str, Tool] = {}
        self._history: Deque[Dict[str, str]] = deque(maxlen=2 * config.LLM_HISTORY_TURNS)

    def _ensure_tools(self) -> None:
        if not self._tools:
            self._tools = build_tools(self._client.get_openapi())
            _log.info(f'[AGENT] {len(self._tools)} tools loaded from the dashboard API')

    def run(self, text: str) -> AgentResult:
        try:
            self._ensure_tools()
        except ApiClientError as exc:
            return AgentResult(f"I can't reach the robot's API: {exc.message}")

        messages: List[Dict[str, Any]] = [
            {'role': 'system', 'content': _SYSTEM}, {'role': 'user', 'content': self._with_history(text)},
        ]
        result = AgentResult(reply='')
        tool_defs = [t.schema for t in self._tools.values()]

        for _ in range(config.LLM_AGENT_MAX_STEPS):
            try:
                resp = requests.post(
                    f'{config.OLLAMA_URL}/api/chat', timeout=config.LLM_AGENT_TIMEOUT_SEC,
                    json={'model': config.LLM_MODEL, 'messages': messages, 'tools': tool_defs,
                          'stream': False, 'keep_alive': -1, 'options': {'temperature': 0}},
                )
                resp.raise_for_status()
                message = resp.json()['message']
            except (requests.exceptions.RequestException, ValueError, KeyError) as exc:
                _log.warning(f'[AGENT] LLM unavailable: {exc}')
                result.reply = result.reply or "I couldn't reach my language model."
                return result

            calls = message.get('tool_calls') or []
            if not calls:
                result.reply = (message.get('content') or '').strip() or 'Done.'
                self._remember(text, result.reply)
                return result

            messages.append(message)
            for call in calls:
                fn = call.get('function', {})
                outcome = self._execute(fn.get('name', ''), fn.get('arguments') or {}, result)
                messages.append({'role': 'tool', 'tool_name': fn.get('name', ''), 'content': outcome})

        result.reply = 'I stopped because that took too many steps.'
        self._remember(text, result.reply)
        return result

    def _with_history(self, text: str) -> str:
        """Earlier turns go INSIDE the user message, not as separate chat
        messages: Llama's template renders the tool list after any prior
        messages, so history before it changes the start of the prompt
        every turn and forces Ollama to re-read all the tools (~35 s on
        CPU). In the user text it stays after the tools, which keep a
        stable, cached start (~2 s)."""
        if not self._history:
            return text
        lines = [f"{'user said' if m['role'] == 'user' else 'you replied'}: {m['content']!r}" for m in self._history]
        return 'Earlier in this conversation: ' + '; '.join(lines) + f'.\n\nNow: {text}'

    def warm_up(self) -> None:
        """Load the model and cache the tool prompt now (one dummy request),
        so the first real command doesn't pay the ~30 s cold cost.
        Safe to call from a background thread; failures are ignored."""
        try:
            self._ensure_tools()
            requests.post(
                f'{config.OLLAMA_URL}/api/chat', timeout=config.LLM_AGENT_TIMEOUT_SEC * 4,
                json={'model': config.LLM_MODEL, 'messages': [
                    {'role': 'system', 'content': _SYSTEM}, {'role': 'user', 'content': 'hello'}],
                    'tools': [t.schema for t in self._tools.values()], 'stream': False,
                    'keep_alive': -1, 'options': {'temperature': 0, 'num_predict': 1}},
            ).raise_for_status()
            _log.info('[AGENT] Model warmed up')
        except (requests.exceptions.RequestException, ApiClientError) as exc:
            _log.warning(f'[AGENT] Warm-up skipped: {exc}')

    def _remember(self, text: str, reply: str) -> None:
        self._history.append({'role': 'user', 'content': text})
        self._history.append({'role': 'assistant', 'content': reply})

    def _execute(self, name: str, args: Any, result: AgentResult) -> str:
        tool = self._tools.get(name)
        if tool is None:
            return json.dumps({'error': f'Unknown tool {name}.'})
        if isinstance(args, str):
            try:
                args = json.loads(args)
            except ValueError:
                return json.dumps({'error': 'Arguments must be a JSON object.'})
        args = {k: v for k, v in args.items() if v is not None}  # models pass null for "omitted"
        missing = [p for p in tool.path_params if p not in args]
        if missing:
            return json.dumps({'error': f'Missing required argument(s): {missing}'})

        for method, prefix in _blocked_rules():
            if tool.method == method and tool.path.startswith(prefix):
                _log.warning(f'[AGENT] Blocked {tool.method} {tool.path}')
                return json.dumps({'error': 'This operation is blocked for voice control. Tell the user to do it in the dashboard.'})

        path = tool.path
        for p in tool.path_params:
            path = path.replace('{' + p + '}', quote(str(args[p]), safe=''))
        params = {k: args[k] for k in tool.query_params if k in args}
        body = {k: args[k] for k in tool.body_params if k in args}

        _log.info(f'[AGENT] {tool.method} {path} params={params} body={body}')
        if self._dry_run and tool.method != 'GET':
            _log.info('[DRY RUN] not sent')
            result.calls.append((tool.method, path))
            return json.dumps({'dry_run': True, 'note': 'Not sent (dry run); assume it succeeded.'})
        try:
            data = self._client.call(
                tool.method, path, params=params, body=body if tool.body_params else None,
                timeout=_MOVE_TIMEOUT_SEC if tool.method != 'GET' else None,
            )
        except ApiClientError as exc:
            _log.warning(f'[AGENT] API error: {exc.message}')
            return json.dumps({'error': exc.message})
        result.calls.append((tool.method, path))
        return json.dumps(data)[:_MAX_RESULT_CHARS]


def started_program(calls: List[Tuple[str, str]]) -> Optional[str]:
    """Name of the last program the agent started, if any (so the node can
    announce when it finishes)."""
    from urllib.parse import unquote
    for method, path in reversed(calls):
        m = _RUN_RE.match(path)
        if method == 'POST' and m:
            return unquote(m.group(1))
    return None
