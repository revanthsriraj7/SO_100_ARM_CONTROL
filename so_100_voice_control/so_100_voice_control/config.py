"""All tunables for the voice-control pipeline, read from environment
variables with sane defaults -- nothing below is hardcoded into the
modules that use it (microphone.py, fast_whisper.py, robot_client.py,
voice_control_node.py). Mirrors the pattern so101_dashboard/config uses
for its own config.yaml: one place, nowhere else hardcodes these values.

Env vars:
    SO101_API_URL     -- base URL of the dashboard's REST API.
    WHISPER_MODEL      -- faster-whisper model size/name (e.g. "base", "small").
    WHISPER_DEVICE      -- "cpu" or "cuda".
    WHISPER_COMPUTE_TYPE -- faster-whisper compute_type (e.g. "int8", "float16").
    MIC_DEVICE          -- sounddevice input device index or name substring.
                            Leave unset to keep using the OS-default input
                            device (this is what the original microphone.py
                            did, and is normally already the paired
                            Bluetooth mic once it's the OS default).
    RECORD_DURATION     -- seconds to record for the fixed-length
                            record_audio() (used by --once/--text testing
                            and the standalone `python microphone.py`
                            script only -- the background listening loop
                            uses the VAD_* settings below instead).
    SAMPLE_RATE          -- recording sample rate, Hz.
    VAD_ENERGY_THRESHOLD  -- RMS level (int16 scale, 0-32767) above which a
                              frame counts as speech. Background/room noise
                              floor is typically well under 200; a headset
                              mic close to your mouth is typically well
                              over 1000 when you're talking. Tune this to
                              your mic/environment if it's mis-triggering
                              or missing quiet speech.
    VAD_FRAME_MS          -- analysis frame size, ms.
    VAD_SILENCE_DURATION_SEC -- how much continuous silence ends an utterance.
    VAD_MIN_SPEECH_DURATION_SEC -- utterances shorter than this (a cough,
                                    a click) are discarded, never transcribed.
    VAD_MAX_UTTERANCE_DURATION_SEC -- safety cap so a mis-tuned threshold
                                        can't record forever.
    VAD_PRE_ROLL_MS       -- audio kept from just before speech was
                               detected, so the first word isn't clipped.
    VOICE_DRY_RUN        -- "true"/"1" to log intended API calls instead of
                              sending them.
    VOICE_TTS_ENABLED    -- "true"/"1" to speak responses out loud.
    TTS_ENGINE            -- "auto" (default: Piper if a voice model is
                              found, else espeak), "piper", or "espeak" to
                              force one.
    TTS_VOICE_MODEL        -- path to a Piper .onnx voice model (its
                                .onnx.json config is expected alongside
                                it). Piper is a small local neural TTS --
                                far more natural than espeak's formant
                                synthesis, still fully offline at
                                inference time. Get voices from
                                https://github.com/rhasspy/piper/blob/master/VOICES.md
                                (~60MB per voice); defaults to
                                ~/.local/share/piper-voices/en_US-lessac-medium.onnx.
    SO101_PROGRAM_PICK_BOBBIN_UP / _TURN_BOBBIN_ON / _TURN_BOBBIN_OFF /
    _REMOVE_BOBBIN -- override the saved dashboard program name each
                       bobbin-sequence voice intent maps to, in case any
                       of them is saved under a different name.
"""
from __future__ import annotations

import os


def _bool_env(name: str, default: bool) -> bool:
    value = os.environ.get(name)
    if value is None:
        return default
    return value.strip().lower() in ('1', 'true', 'yes', 'on')


# -- dashboard API --
API_URL: str = os.environ.get('SO101_API_URL', 'http://127.0.0.1:8000')
API_TIMEOUT_SEC: float = float(os.environ.get('SO101_API_TIMEOUT', '10.0'))

# -- microphone --
MIC_DEVICE = os.environ.get('MIC_DEVICE') or None
SAMPLE_RATE: int = int(os.environ.get('SAMPLE_RATE', '16000'))
RECORD_DURATION: float = float(os.environ.get('RECORD_DURATION', '5.0'))

# -- voice-activity detection (background listening) --
VAD_ENERGY_THRESHOLD: float = float(os.environ.get('VAD_ENERGY_THRESHOLD', '500'))
VAD_FRAME_MS: int = int(os.environ.get('VAD_FRAME_MS', '30'))
VAD_SILENCE_DURATION_SEC: float = float(os.environ.get('VAD_SILENCE_DURATION_SEC', '0.8'))
VAD_MIN_SPEECH_DURATION_SEC: float = float(os.environ.get('VAD_MIN_SPEECH_DURATION_SEC', '0.3'))
VAD_MAX_UTTERANCE_DURATION_SEC: float = float(os.environ.get('VAD_MAX_UTTERANCE_DURATION_SEC', '15.0'))
VAD_PRE_ROLL_MS: int = int(os.environ.get('VAD_PRE_ROLL_MS', '300'))

# -- whisper --
WHISPER_MODEL: str = os.environ.get('WHISPER_MODEL', 'base')
WHISPER_DEVICE: str = os.environ.get('WHISPER_DEVICE', 'cpu')
WHISPER_COMPUTE_TYPE: str = os.environ.get('WHISPER_COMPUTE_TYPE', 'int8')
WHISPER_LANGUAGE: str = os.environ.get('WHISPER_LANGUAGE', 'en')

# -- program name mapping (voice intent -> actual saved program name) --
# The bobbin workflow is four separate dashboard programs, run as
# individual steps rather than one combined program -- each gets its own
# voice intent (see voice_commands.py) so they can be run/navigated
# independently, plus a "next step" intent that walks PROGRAM_SEQUENCE in
# order without needing to name each one.
PROGRAM_NAMES = {
    'pick_bobbin_up': os.environ.get('SO101_PROGRAM_PICK_BOBBIN_UP', 'pick_bobbin_up'),
    'turn_bobbin_on': os.environ.get('SO101_PROGRAM_TURN_BOBBIN_ON', 'turn_bobbin_on'),
    'turn_bobbin_off': os.environ.get('SO101_PROGRAM_TURN_BOBBIN_OFF', 'turn_bobbin_off'),
    'remove_bobbin': os.environ.get('SO101_PROGRAM_REMOVE_BOBBIN', 'remove_bobbin'),
}

# The order "next step" advances through -- matches how the four programs
# are meant to run in sequence: pick the bobbin up, turn the winder on,
# turn it off once wound, remove the finished bobbin.
PROGRAM_SEQUENCE = [
    PROGRAM_NAMES['pick_bobbin_up'],
    PROGRAM_NAMES['turn_bobbin_on'],
    PROGRAM_NAMES['turn_bobbin_off'],
    PROGRAM_NAMES['remove_bobbin'],
]

# Spoken form of each program, for TTS status reports ("Pick bobbin up
# complete.") -- kept separate from PROGRAM_NAMES since the saved program
# name (used in API calls) and how it should be *said* aren't always the
# same string.
PROGRAM_SPOKEN_NAMES = {
    PROGRAM_NAMES['pick_bobbin_up']: 'Pick bobbin up',
    PROGRAM_NAMES['turn_bobbin_on']: 'Turn bobbin on',
    PROGRAM_NAMES['turn_bobbin_off']: 'Turn bobbin off',
    PROGRAM_NAMES['remove_bobbin']: 'Remove bobbin',
}

# -- runtime flags --
DRY_RUN: bool = _bool_env('VOICE_DRY_RUN', False)
TTS_ENABLED: bool = _bool_env('VOICE_TTS_ENABLED', True)

# -- text-to-speech --
TTS_ENGINE: str = os.environ.get('TTS_ENGINE', 'auto')
TTS_VOICE_MODEL: str = os.environ.get(
    'TTS_VOICE_MODEL',
    os.path.expanduser('~/.local/share/piper-voices/jarvis-high.onnx'),
)

# -- program status polling (Phase 5) --
STATUS_POLL_INTERVAL_SEC: float = float(os.environ.get('SO101_STATUS_POLL_INTERVAL', '1.0'))
STATUS_POLL_TIMEOUT_SEC: float = float(os.environ.get('SO101_STATUS_POLL_TIMEOUT', '300.0'))
