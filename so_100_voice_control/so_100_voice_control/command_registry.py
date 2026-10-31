"""Loads voice-command definitions from YAML files and matches transcripts
against them. Nothing workflow-specific lives in Python: program names,
phrases, spoken names, LLM hints and "next step" sequences all come from
the files in commands/ (and VOICE_COMMANDS_DIR, if set), so adding a
command means editing YAML, not code.

File shapes (see commands/core.yaml and commands/bobbin.yaml):
    actions:   {<built-in action>: [phrases]}
    workflow:  <name>
    programs:  {<dashboard program name>: {phrases, spoken, description}}
    targets:   {<dashboard target name>: {phrases}}
    sequence:  [<program name>, ...]   # order "next step" walks through

Matching is whole-word, case- and punctuation-insensitive, and the longest
matching phrase wins -- so file order and phrase order never matter.
"""
from __future__ import annotations

import re
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import yaml

from . import config
from .utils import get_logger

_log = get_logger('command_registry')

BUILTIN_DIR = Path(__file__).parent / 'commands'
_NON_WORD = re.compile(r"[^a-z0-9 ]+")

# Built-in actions a YAML file may bind phrases to (mirrors voice_commands).
ACTIONS = {'next_step', 'stop_program', 'go_home', 'jaw_open', 'jaw_close', 'get_status'}


class CommandConfigError(ValueError):
    """A commands YAML file is malformed or contradicts another."""


def normalize(text: str) -> str:
    text = _NON_WORD.sub('', (text or '').lower().replace('-', ' '))
    return ' '.join(text.split())


@dataclass(frozen=True)
class Match:
    action: str
    program: Optional[str] = None
    target: Optional[str] = None


@dataclass
class Workflow:
    name: str
    sequence: List[str] = field(default_factory=list)


@dataclass
class CommandRegistry:
    # normalized phrase -> Match
    phrases: Dict[str, Match] = field(default_factory=dict)
    spoken: Dict[str, str] = field(default_factory=dict)
    descriptions: Dict[str, str] = field(default_factory=dict)
    workflows: Dict[str, Workflow] = field(default_factory=dict)

    def match(self, text: str) -> Optional[Match]:
        padded = f' {normalize(text)} '
        best: Optional[Tuple[int, Match]] = None
        for phrase, m in self.phrases.items():
            if f' {phrase} ' in padded and (best is None or len(phrase) > best[0]):
                best = (len(phrase), m)
        return best[1] if best else None

    def spoken_name(self, program: str) -> str:
        return self.spoken.get(program, program.replace('_', ' '))

    def locate(self, program: str) -> Optional[Tuple[str, int]]:
        """(workflow name, index) of `program` in a sequence, if any."""
        for wf in self.workflows.values():
            if program in wf.sequence:
                return wf.name, wf.sequence.index(program)
        return None

    def default_workflow(self) -> Optional[str]:
        return next((n for n, wf in self.workflows.items() if wf.sequence), None)

    # -- loading --

    def _add_phrases(self, source: str, phrases: List[str], m: Match) -> None:
        for raw in phrases:
            key = normalize(str(raw))
            if not key:
                raise CommandConfigError(f'{source}: empty phrase for {m}')
            existing = self.phrases.get(key)
            if existing is not None and existing != m:
                raise CommandConfigError(
                    f'{source}: phrase "{raw}" is already mapped to {existing}, cannot also map to {m}'
                )
            self.phrases[key] = m

    def load_file(self, path: Path) -> None:
        src = path.name
        try:
            data = yaml.safe_load(path.read_text()) or {}
        except yaml.YAMLError as exc:
            raise CommandConfigError(f'{src}: invalid YAML: {exc}') from exc
        if not isinstance(data, dict):
            raise CommandConfigError(f'{src}: top level must be a mapping')
        unknown = set(data) - {'workflow', 'actions', 'programs', 'targets', 'sequence'}
        if unknown:
            raise CommandConfigError(f'{src}: unknown keys {sorted(unknown)}')

        for action, phrases in (data.get('actions') or {}).items():
            if action not in ACTIONS:
                raise CommandConfigError(f'{src}: unknown action "{action}" (valid: {sorted(ACTIONS)})')
            self._add_phrases(src, phrases or [], Match(action=action))

        programs = data.get('programs') or {}
        for name, spec in programs.items():
            spec = spec or {}
            self._add_phrases(src, spec.get('phrases') or [], Match(action='run_program', program=name))
            if spec.get('spoken'):
                self.spoken[name] = spec['spoken']
            if spec.get('description'):
                self.descriptions[name] = spec['description']

        for name, spec in (data.get('targets') or {}).items():
            self._add_phrases(src, (spec or {}).get('phrases') or [], Match(action='move_target', target=name))

        sequence = data.get('sequence') or []
        missing = [p for p in sequence if p not in programs]
        if missing:
            raise CommandConfigError(f'{src}: sequence lists programs not defined in this file: {missing}')
        if sequence:
            wf_name = data.get('workflow') or path.stem
            if wf_name in self.workflows:
                raise CommandConfigError(f'{src}: duplicate workflow name "{wf_name}"')
            self.workflows[wf_name] = Workflow(wf_name, list(sequence))


def load_registry(extra_dir: Optional[str] = None) -> CommandRegistry:
    """Load every *.yaml in the built-in commands/ folder, then in
    `extra_dir` (default: $VOICE_COMMANDS_DIR) if given, in sorted order."""
    registry = CommandRegistry()
    dirs = [BUILTIN_DIR]
    extra = extra_dir if extra_dir is not None else config.COMMANDS_DIR
    if extra:
        dirs.append(Path(extra).expanduser())
    for d in dirs:
        if not d.is_dir():
            raise CommandConfigError(f'Commands directory not found: {d}')
        for path in sorted(d.glob('*.yaml')):
            registry.load_file(path)
            _log.info(f'[VOICE] Loaded commands from {path}')
    return registry


_registry: Optional[CommandRegistry] = None


def get_registry() -> CommandRegistry:
    global _registry
    if _registry is None:
        _registry = load_registry()
    return _registry
