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
    VOICE_COMMANDS_DIR   -- extra folder of command *.yaml files, loaded
                             after the built-in commands/ (see
                             command_registry.py for the file format).
    VOICE_LLM_AGENT / LLM_AGENT_MAX_STEPS / LLM_AGENT_TIMEOUT /
    LLM_BLOCKED_OPS      -- tool-calling agent over the dashboard API
                             (llm_agent.py).
    VOICE_LLM_ENABLED / OLLAMA_URL / LLM_MODEL / LLM_TIMEOUT /
    LLM_HISTORY_TURNS    -- local Llama fallback for transcripts the
                             phrase table does not recognize (llm_parser.py).
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

# -- voice command definitions --
# Built-in commands/*.yaml are always loaded; set this to an extra folder of
# *.yaml files to add or extend commands without touching the package.
COMMANDS_DIR = os.environ.get('VOICE_COMMANDS_DIR') or None

# -- local LLM intent parser (Ollama) --
# The phrase table in voice_commands.py is tried first; the LLM is only
# consulted for transcripts it doesn't recognize.
LLM_ENABLED: bool = _bool_env('VOICE_LLM_ENABLED', True)
OLLAMA_URL: str = os.environ.get('OLLAMA_URL', 'http://127.0.0.1:11434')
LLM_MODEL: str = os.environ.get('LLM_MODEL', 'llama3.2:3b')
LLM_TIMEOUT_SEC: float = float(os.environ.get('LLM_TIMEOUT', '20.0'))
LLM_HISTORY_TURNS: int = int(os.environ.get('LLM_HISTORY_TURNS', '3'))

# -- LLM agent (tool-calling over the dashboard's OpenAPI) --
# OFF by default: on a CPU-only laptop it takes ~35 s per command (see
# llm_agent.py), so the preset command list + fixed-action parser is used.
# When on, anything the emergency fast path doesn't catch goes to an Ollama
# model that is handed every dashboard endpoint as a tool and decides
# what to call itself. Off = the old fixed-action parser.
LLM_AGENT_ENABLED: bool = _bool_env('VOICE_LLM_AGENT', False)
LLM_AGENT_MAX_STEPS: int = int(os.environ.get('LLM_AGENT_MAX_STEPS', '8'))
LLM_AGENT_TIMEOUT_SEC: float = float(os.environ.get('LLM_AGENT_TIMEOUT', '60.0'))
# Operations the model may never call, "METHOD /path-prefix" comma list.
# Default: going limp (free move) and anything that deletes -- do those
# by hand in the dashboard. Set empty ("") to allow everything.
LLM_BLOCKED_OPS: str = os.environ.get(
    'LLM_BLOCKED_OPS', 'POST /api/robot/torque/disable,DELETE /',
)

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
