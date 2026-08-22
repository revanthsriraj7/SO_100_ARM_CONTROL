"""Deterministic transcript -> validated command parser.

transcribed text -> recognized intent -> validated Command

This is intentionally NOT an LLM. It's a small alias table matched
against the lowercased transcript. The safety property that matters:
anything that doesn't match a known phrase becomes action="unknown" and
is never turned into a robot action -- there is no fallback path that
lets arbitrary text reach the robot.

Kept modular on purpose (see module docstring goal in the integration
spec): a future LLM-based parser only needs to produce the same Command
shape (action + optional program) from a transcript; robot_client.py and
voice_control_node.py never need to change.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional, Tuple

from . import config

# -- supported actions --
ACTION_RUN_PROGRAM = 'run_program'
ACTION_NEXT_STEP = 'next_step'
ACTION_STOP_PROGRAM = 'stop_program'
ACTION_GO_HOME = 'go_home'
ACTION_JAW_OPEN = 'jaw_open'
ACTION_JAW_CLOSE = 'jaw_close'
ACTION_GET_STATUS = 'get_status'
ACTION_UNKNOWN = 'unknown'


@dataclass
class Command:
    action: str
    program: Optional[str] = None
    raw_text: str = ''

    def as_dict(self) -> dict:
        d = {'action': self.action}
        if self.program is not None:
            d['program'] = self.program
        return d


# Each entry: (action, program_key_or_None, [phrases]). Matched by
# substring against the lowercased transcript, in order -- first match
# wins, so more specific phrases should be listed first if they overlap.
_RULES: List[Tuple[str, Optional[str], List[str]]] = [
    # The bobbin workflow is four separate dashboard programs, run one at
    # a time -- these four rules must come before ACTION_STOP_PROGRAM
    # below since "turn off ..." would otherwise never be reached if a
    # generic "stop"-ish phrase matched first (it doesn't currently, but
    # keep this ordering if that rule's phrase list ever grows).
    (ACTION_RUN_PROGRAM, 'remove_bobbin', [
        'remove the bobbin',
        'remove bobbin',
        'take out the bobbin',
        'take the bobbin out',
        'unload the bobbin',
    ]),
    (ACTION_RUN_PROGRAM, 'turn_bobbin_off', [
        'turn off the bobbin',
        'turn the bobbin off',
        'turn off the bobbin machine',
        'turn off the machine',
        'switch off the bobbin',
        'stop the machine',
    ]),
    (ACTION_RUN_PROGRAM, 'turn_bobbin_on', [
        'turn on the bobbin',
        'turn the bobbin on',
        'turn on the bobbin machine',
        'turn on the machine',
        'switch on the bobbin',
        'start the bobbin machine',
        'start the machine',
    ]),
    (ACTION_RUN_PROGRAM, 'pick_bobbin_up', [
        'pick up the bobbin',
        'pick bobbin up',
        'pick up bobbin',
        'grab the bobbin',
        # Legacy/generic phrasing from before the workflow was split into
        # four steps -- treated as "start the sequence from the top".
        'start bobbin winding',
        'start winding',
        'wind a bobbin',
        'wind bobbin',
        'begin bobbin winding',
        'bobbin winding',
        'pick bobbin',
    ]),
    (ACTION_NEXT_STEP, None, [
        'next step',
        'next bobbin step',
        'move to the next step',
        'continue',
        'next',
    ]),
    (ACTION_STOP_PROGRAM, None, [
        'stop the robot',
        'stop the program',
        'abort',
        'stop',
    ]),
    (ACTION_GO_HOME, None, [
        'go home',
        'home the robot',
        'return home',
        'move home',
    ]),
    (ACTION_JAW_OPEN, None, [
        'open the gripper',
        'open gripper',
        'open the jaw',
        'open jaw',
    ]),
    (ACTION_JAW_CLOSE, None, [
        'close the gripper',
        'close gripper',
        'close the jaw',
        'close jaw',
    ]),
    (ACTION_GET_STATUS, None, [
        'what is the robot doing',
        "what's the robot doing",
        'robot status',
        "what's happening",
        'what is happening',
    ]),
]


def parse_command(text: str) -> Command:
    """Match `text` against the known phrase table and return a Command.
    Returns Command(action=ACTION_UNKNOWN, raw_text=text) if nothing
    matches -- callers must treat that as "do not touch the robot"."""
    normalized = (text or '').strip().lower()
    if not normalized:
        return Command(action=ACTION_UNKNOWN, raw_text=text)

    for action, program_key, phrases in _RULES:
        for phrase in phrases:
            if phrase in normalized:
                program = config.PROGRAM_NAMES[program_key] if program_key else None
                return Command(action=action, program=program, raw_text=text)

    return Command(action=ACTION_UNKNOWN, raw_text=text)
