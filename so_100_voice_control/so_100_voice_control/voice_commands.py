"""Deterministic transcript -> validated command parser.

transcribed text -> recognized intent -> validated Command

This is intentionally NOT an LLM. It's a phrase table, loaded from the
YAML files in commands/ (see command_registry.py), matched against the
transcript. The safety property that matters:
anything that doesn't match a known phrase becomes action="unknown" and
is never turned into a robot action -- there is no fallback path that
lets arbitrary text reach the robot.

The LLM parser (llm_parser.py) produces the same Command shape and is only
consulted when nothing here matches.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

from . import command_registry

# -- supported actions --
ACTION_RUN_PROGRAM = 'run_program'
ACTION_NEXT_STEP = 'next_step'
ACTION_STOP_PROGRAM = 'stop_program'
ACTION_GO_HOME = 'go_home'
ACTION_JAW_OPEN = 'jaw_open'
ACTION_JAW_CLOSE = 'jaw_close'
ACTION_GET_STATUS = 'get_status'
ACTION_MOVE_TARGET = 'move_target'
ACTION_UNKNOWN = 'unknown'


@dataclass
class Command:
    action: str
    program: Optional[str] = None
    target: Optional[str] = None
    reply: Optional[str] = None
    raw_text: str = ''

    def as_dict(self) -> dict:
        d = {'action': self.action}
        if self.program is not None:
            d['program'] = self.program
        if self.target is not None:
            d['target'] = self.target
        return d


def parse_command(text: str) -> Command:
    """Match `text` against the phrases loaded from commands/*.yaml and
    return a Command. Returns Command(action=ACTION_UNKNOWN, raw_text=text)
    if nothing matches -- callers must treat that as "do not touch the
    robot"."""
    match = command_registry.get_registry().match(text)
    if match is None:
        return Command(action=ACTION_UNKNOWN, raw_text=text)
    return Command(action=match.action, program=match.program, target=match.target, raw_text=text)
