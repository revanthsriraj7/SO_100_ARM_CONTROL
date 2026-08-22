"""Entry point for the voice-control pipeline:

    Bluetooth mic -> microphone.BackgroundListener -> fast_whisper.transcribe()
    -> voice_commands.parse_command() -> SO101Client -> dashboard API
    -> existing program executor -> tts.speak() for feedback

This module owns the LISTEN -> TRANSCRIBE -> PROCESS -> EXECUTE ->
RETURN TO LISTENING cycle and nothing else: it never touches ROS 2,
MoveIt, or any hardware/manager object directly. It doesn't need an
rclpy.Node -- every dashboard-facing action already goes through the
existing REST API (see robot_client.py), and this package publishes no
topics/services of its own, so wrapping it in a Node would just be
ceremony with no ROS behavior behind it. `ros2 run so_100_voice_control
voice_control` still works because this is a plain ament_python
console_script entry point, which ros2 run supports regardless of
whether the target uses rclpy.

LISTENING MODEL: the mic stays open continuously (microphone.
BackgroundListener) rather than recording a fixed window every cycle and
always demanding a command out of it -- VAD decides when someone actually
spoke, and only a detected utterance ever reaches Whisper/the parser. An
empty or unrecognizable transcript from a real utterance still gets
"I don't recognize that robot command."; silence never does, because
silence never produces an utterance in the first place.

SAFETY: parse_command() only ever returns one of a small whitelist of
actions (see voice_commands.py). Anything else comes back as
action="unknown" and is refused here -- nothing here ever calls the API
for an unrecognized command, no matter what the transcript says.
"""
from __future__ import annotations

import argparse
import sys
import time
from typing import Optional

import numpy as np

from . import config, voice_commands
from .fast_whisper import transcribe
from .microphone import BackgroundListener, save_wav
from .robot_client import ApiClientError, SO101Client
from .tts import speak
from .utils import get_logger

_log = get_logger('voice_control_node')

_TERMINAL_STATES = {'FINISHED', 'STOPPED', 'FAILED'}
_STATUS_PHRASES = {
    'FINISHED': 'complete',
    'STOPPED': 'stopped',
    'FAILED': 'failed',
}


class VoiceControlNode:
    def __init__(self, client: Optional[SO101Client] = None, dry_run: Optional[bool] = None) -> None:
        self.client = client or SO101Client()
        self.dry_run = config.DRY_RUN if dry_run is None else dry_run
        self._listener: Optional[BackgroundListener] = None
        # Index into config.PROGRAM_SEQUENCE of the last bobbin-sequence
        # program run (by name or via "next step") -- -1 means the
        # sequence hasn't been started yet this session. Only advanced by
        # actually running a step, so saying the same step's name twice
        # doesn't skip anything, and "next step" always means "the one
        # after whatever last ran".
        self._sequence_index = -1

    # -- startup checks (Phase 7, steps 2-3) --

    def check_ready(self) -> bool:
        _log.info(f'[VOICE] Checking dashboard API at {self.client.base_url} ...')
        if not self.client.is_reachable():
            _log.error(f'[VOICE] Dashboard API is not reachable at {self.client.base_url}.')
            return False
        try:
            status = self.client.get_status()
        except ApiClientError as exc:
            _log.error(f'[VOICE] Could not read robot status: {exc.message}')
            return False
        _log.info(f'[API] status: {status}')
        if not status.get('controller_active', False) and not status.get('mock_mode', False):
            _log.warning('[VOICE] Robot controller is not active yet -- commands may be rejected until it is.')
        return True

    # -- listen/execute cycles --

    def run_once(self, text: Optional[str] = None) -> None:
        """Run one LISTEN -> ... -> EXECUTE cycle and return. Pass `text`
        to skip audio capture entirely and test parsing/execution
        directly (see --text on the CLI). Without `text`, this opens the
        mic just long enough to catch one VAD-detected utterance (it
        blocks until you actually say something, rather than recording a
        fixed window) -- for a persistent session use run_forever()
        instead."""
        if text is not None:
            self._handle_transcript(text)
            return

        _log.info('[VOICE] Listening for one utterance (speak now)...')
        listener = BackgroundListener()
        listener.start()
        try:
            audio = None
            while audio is None:
                audio = listener.get_utterance(timeout=1.0)
        finally:
            listener.stop()

        transcript = self._transcribe(audio)
        if transcript is not None:
            self._handle_transcript(transcript)

    def run_forever(self) -> None:
        """Keeps the mic open continuously. Nothing is transcribed, and
        nothing is said back, unless VAD actually detected speech -- so
        this doesn't nag for a command every few seconds the way a fixed
        record-then-guess cycle would."""
        self._listener = BackgroundListener()
        self._listener.start()
        try:
            while True:
                audio = self._listener.get_utterance(timeout=1.0)
                if audio is None:
                    continue  # nothing said in this window -- keep waiting quietly
                try:
                    transcript = self._transcribe(audio)
                    if transcript is not None:
                        self._handle_transcript(transcript)
                except Exception as exc:  # noqa: BLE001 -- one bad utterance must not kill the listening loop
                    _log.error(f'[VOICE] Error handling utterance: {exc}')
        finally:
            self._listener.stop()
            self._listener = None

    def _transcribe(self, audio: np.ndarray) -> Optional[str]:
        path = save_wav(audio)
        text = transcribe(path)
        if not text or not text.strip():
            # VAD heard *something* above the energy threshold (a knock, a
            # cough, background noise), but Whisper found no actual words
            # in it -- discard quietly, same as silence. Only a transcript
            # with real words reaches the parser/TTS.
            _log.info('[VOICE] (heard something, but no recognizable speech -- ignoring)')
            return None
        _log.info(f'[VOICE] Transcription: "{text}"')
        return text

    def _handle_transcript(self, text: str) -> None:
        command = voice_commands.parse_command(text)
        _log.info(f'[VOICE] Intent: {command.action}')
        if command.program:
            _log.info(f'[VOICE] Program: {command.program}')
        self._execute(command)

    # -- dispatch --

    def _speak(self, text: str) -> None:
        """Routes every spoken response through here so the background
        listener is paused first -- otherwise the mic would pick up the
        robot's own TTS output (especially likely on a Bluetooth headset)
        and could loop back in as a bogus "command"."""
        if self._listener is not None:
            self._listener.pause()
        try:
            speak(text)
        finally:
            if self._listener is not None:
                self._listener.resume()

    def _execute(self, command: voice_commands.Command) -> None:
        action = command.action

        if action == voice_commands.ACTION_UNKNOWN:
            self._speak("I don't recognize that robot command.")
            return

        if action == voice_commands.ACTION_RUN_PROGRAM:
            self._run_program(command.program)
        elif action == voice_commands.ACTION_NEXT_STEP:
            self._next_step()
        elif action == voice_commands.ACTION_STOP_PROGRAM:
            self._call('Stopping the program.', self.client.stop_program)
        elif action == voice_commands.ACTION_GO_HOME:
            self._call('Going home.', self.client.go_home)
        elif action == voice_commands.ACTION_JAW_OPEN:
            self._call('Opening the gripper.', self.client.open_gripper)
        elif action == voice_commands.ACTION_JAW_CLOSE:
            self._call('Closing the gripper.', self.client.close_gripper)
        elif action == voice_commands.ACTION_GET_STATUS:
            self._report_status()
        else:
            # Defensive: parse_command() should never produce anything
            # outside the whitelist above, but refuse rather than guess.
            self._speak("I don't recognize that robot command.")

    def _call(self, spoken_ack: str, fn, *args) -> None:
        self._speak(spoken_ack)
        if self.dry_run:
            _log.info(f'[DRY RUN]\nWould execute: {fn.__name__}({", ".join(map(repr, args))})')
            return
        _log.info('[VOICE] Sending request...')
        try:
            result = fn(*args)
            _log.info(f'[API] {result}')
        except ApiClientError as exc:
            _log.error(f'[API] Error: {exc.message}')
            self._speak(f'Sorry, that failed: {exc.message}')

    def _next_step(self) -> None:
        """ACTION_NEXT_STEP: run the bobbin-sequence program after
        whichever one last ran (or the first one, if none has run yet
        this session), so the four steps can be driven hands-free without
        naming each one -- "navigate through" the sequence."""
        sequence = config.PROGRAM_SEQUENCE
        next_index = self._sequence_index + 1
        if next_index >= len(sequence):
            self._speak('That was the last step. Say "pick up the bobbin" to start over.')
            return
        self._run_program(sequence[next_index], sequence_index=next_index)

    def _run_program(self, program: str, sequence_index: Optional[int] = None) -> None:
        """sequence_index is passed when this program is a step of
        config.PROGRAM_SEQUENCE (whether reached by name or via "next
        step") so _next_step() knows what comes after it; left None for
        programs outside that sequence."""
        if sequence_index is None and program in config.PROGRAM_SEQUENCE:
            sequence_index = config.PROGRAM_SEQUENCE.index(program)

        if self.dry_run:
            self._speak(f'Starting {self._spoken_name(program)}.')
            _log.info(f'[DRY RUN]\nWould execute: run_program({program!r})')
            if sequence_index is not None:
                self._sequence_index = sequence_index
            return

        # Reuse the dashboard's own validation (program exists / not
        # already busy / robot connected) rather than duplicating it here
        # -- run_program() below just forwards to POST
        # /api/programs/{name}/run and surfaces whatever the API says.
        self._speak(f'Starting {self._spoken_name(program)}.')
        _log.info('[VOICE] Sending request...')
        try:
            result = self.client.run_program(program)
        except ApiClientError as exc:
            _log.error(f'[API] Error: {exc.message}')
            self._speak(f'Could not start {self._spoken_name(program)}: {exc.message}')
            return

        if sequence_index is not None:
            self._sequence_index = sequence_index
        _log.info(f'[API] {result.get("message", "Program started")}')
        self._monitor_program(program)

    def _monitor_program(self, program: str) -> None:
        """Poll the dashboard's existing program-state endpoint (Phase 5)
        -- no separate program-state system is kept here."""
        deadline = time.monotonic() + config.STATUS_POLL_TIMEOUT_SEC
        last_state = None
        while time.monotonic() < deadline:
            try:
                state_info = self.client.get_state().get('program', {})
            except ApiClientError as exc:
                _log.error(f'[API] Error polling status: {exc.message}')
                return
            state = state_info.get('state')
            if state != last_state:
                _log.info(f'[VOICE] Robot status: {state}')
                last_state = state
            if state in _TERMINAL_STATES:
                phrase = _STATUS_PHRASES.get(state, state.lower())
                self._speak(f'{self._spoken_name(program)} {phrase}.')
                return
            time.sleep(config.STATUS_POLL_INTERVAL_SEC)
        _log.warning(f'[VOICE] Timed out waiting for {program} to finish.')

    def _report_status(self) -> None:
        try:
            state = self.client.get_state()
        except ApiClientError as exc:
            self._speak(f"I couldn't read the robot's status: {exc.message}")
            return
        program = state.get('program', {})
        prog_name = program.get('name')
        prog_state = program.get('state', 'IDLE')
        if prog_state == 'RUNNING' and prog_name:
            self._speak(f'{self._spoken_name(prog_name)} is currently running.')
        else:
            connected = 'connected' if state.get('connected') else 'not connected'
            self._speak(f'The robot is {connected} and idle.')

    @staticmethod
    def _spoken_name(program: str) -> str:
        return config.PROGRAM_SPOKEN_NAMES.get(program, program)


def _build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description='SO-101 voice control: mic -> whisper -> dashboard API.')
    parser.add_argument(
        '--dry-run', action='store_true', default=None,
        help='Log intended dashboard API calls instead of sending them (also settable via VOICE_DRY_RUN=true).',
    )
    parser.add_argument(
        '--once', action='store_true',
        help='Run a single listen/execute cycle and exit, instead of looping forever.',
    )
    parser.add_argument(
        '--text', default=None,
        help='Skip audio recording/transcription and parse this text directly -- for testing the '
             'intent parser (and, combined with --dry-run, the whole pipeline) without a microphone.',
    )
    return parser


def main(argv=None) -> int:
    args = _build_arg_parser().parse_args(argv if argv is not None else sys.argv[1:])
    dry_run = config.DRY_RUN if args.dry_run is None else args.dry_run

    node = VoiceControlNode(dry_run=dry_run)
    if dry_run:
        _log.info('[VOICE] Running in DRY RUN mode -- no dashboard API calls will be sent.')
        # Dry run never calls the API (see VoiceControlNode._call /
        # _run_program), so this also doubles as the "test audio ->
        # transcription -> intent without executing anything" mode the
        # spec asks for -- it works even with the dashboard not running.
        if not node.check_ready():
            _log.warning('[VOICE] Dashboard API not reachable -- continuing anyway, dry run never calls it.')
    elif not node.check_ready():
        _log.error('[VOICE] Dashboard API not reachable -- start so101_dashboard first. Exiting.')
        return 1

    if args.text is not None or args.once:
        node.run_once(text=args.text)
        return 0

    try:
        node.run_forever()
    except KeyboardInterrupt:
        _log.info('[VOICE] Stopped.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
