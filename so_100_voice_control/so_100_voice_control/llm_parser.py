"""Free-form transcript -> validated Command, using a local Llama via Ollama.

Only consulted when the deterministic phrase table in voice_commands.py
finds no match. The safety property is unchanged: the model can only pick
from the same action whitelist, and a program/target name it returns must
exist in the dashboard's live lists or the result is ACTION_UNKNOWN. The
model never produces raw joint/servo values and never sees a path to the
robot except through Command -> VoiceControlNode._execute().

Output is constrained with Ollama's JSON-schema `format`, which is far
more reliable on small models than free-text or native tool calling.
"""
from __future__ import annotations

import json
from collections import deque
from typing import Any, Deque, Dict, List, Optional

import requests

from . import config, voice_commands
from .utils import get_logger

_log = get_logger('llm_parser')

_ACTIONS = [
    voice_commands.ACTION_RUN_PROGRAM,
    voice_commands.ACTION_MOVE_TARGET,
    voice_commands.ACTION_STOP_PROGRAM,
    voice_commands.ACTION_GO_HOME,
    voice_commands.ACTION_JAW_OPEN,
    voice_commands.ACTION_JAW_CLOSE,
    voice_commands.ACTION_GET_STATUS,
    voice_commands.ACTION_UNKNOWN,
]

_SCHEMA = {
    'type': 'object',
    'properties': {
        'action': {'type': 'string', 'enum': _ACTIONS},
        'program': {'type': ['string', 'null']},
        'target': {'type': ['string', 'null']},
        'reply': {'type': 'string'},
    },
    'required': ['action', 'program', 'target', 'reply'],
}

_SYSTEM = """You translate a spoken request into ONE command for a robot arm.
Reply with JSON only. Choose "action" from: {actions}.
- run_program: the request matches a PROGRAM below (a task such as picking, turning on, removing). Set "program" to its exact name. Prefer this whenever a program fits.
- move_target: ONLY if the user asks to move the arm to a TARGET location below. Set "target" to its exact name. Never put a program name in "target".
- stop_program, go_home, jaw_open, jaw_close, get_status: no program/target.
- If the request is unclear, unsafe, or matches nothing below, use "unknown".
Never invent a name. "reply" is a short spoken acknowledgement.

PROGRAMS (for run_program):
{programs}
TARGETS (for move_target): {targets}
Robot state: {state}
"""


class LlmParser:
    def __init__(self) -> None:
        self._history: Deque[Dict[str, str]] = deque(maxlen=2 * config.LLM_HISTORY_TURNS)

    def parse(
        self, text: str, programs: List[str], targets: List[str], state: str,
        descriptions: Optional[Dict[str, str]] = None,
    ) -> voice_commands.Command:
        unknown = voice_commands.Command(action=voice_commands.ACTION_UNKNOWN, raw_text=text)
        system = _SYSTEM.format(
            actions=', '.join(_ACTIONS), programs=self._describe(programs, descriptions or {}),
            targets=', '.join(targets) or '(none)', state=state,
        )
        messages = [{'role': 'system', 'content': system}, *self._history, {'role': 'user', 'content': text}]
        try:
            resp = requests.post(
                f'{config.OLLAMA_URL}/api/chat', timeout=config.LLM_TIMEOUT_SEC,
                json={'model': config.LLM_MODEL, 'messages': messages, 'stream': False,
                      'format': _SCHEMA, 'options': {'temperature': 0}},
            )
            resp.raise_for_status()
            data = json.loads(resp.json()['message']['content'])
        except (requests.exceptions.RequestException, ValueError, KeyError) as exc:
            _log.warning(f'[LLM] Unavailable or bad response, falling back to unknown: {exc}')
            return unknown

        command = self._validate(data, text, programs, targets)
        _log.info(f'[LLM] {data} -> {command.action}')
        self._history.append({'role': 'user', 'content': text})
        self._history.append({'role': 'assistant', 'content': json.dumps(data)})
        return command

    @staticmethod
    def _describe(programs: List[str], descriptions: Dict[str, str]) -> str:
        lines = [f'- {p}: {descriptions[p]}' if p in descriptions else f'- {p}' for p in programs]
        return '\n'.join(lines) or '(none)'

    @staticmethod
    def _validate(data: Dict[str, Any], text: str, programs: List[str], targets: List[str]) -> voice_commands.Command:
        unknown = voice_commands.Command(action=voice_commands.ACTION_UNKNOWN, raw_text=text)
        action = data.get('action')
        if action not in _ACTIONS or action == voice_commands.ACTION_UNKNOWN:
            return unknown
        reply = data.get('reply') or None
        if action == voice_commands.ACTION_RUN_PROGRAM:
            if data.get('program') not in programs:
                return unknown
            return voice_commands.Command(action=action, program=data['program'], reply=reply, raw_text=text)
        if action == voice_commands.ACTION_MOVE_TARGET:
            if data.get('target') not in targets:
                return unknown
            return voice_commands.Command(action=action, target=data['target'], reply=reply, raw_text=text)
        return voice_commands.Command(action=action, reply=reply, raw_text=text)
