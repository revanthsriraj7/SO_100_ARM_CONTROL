"""Local text-to-speech: speak(text).

Two backends, tried in order:
    1. Piper (github.com/rhasspy/piper) -- a small local neural-TTS model.
       Far more natural than formant synthesis, and still fully offline
       at inference time: the ~60MB voice model is a one-time download
       (see config.TTS_VOICE_MODEL), nothing is called out to at
       runtime. Loaded once and reused (get_piper_voice()), same
       load-once-reuse principle as fast_whisper.get_model().
    2. espeak-ng / espeak, shelled out to -- the original fallback. Always
       available if either binary is on PATH, needs no model file, so
       this is what keeps speak() working out of the box even before a
       Piper voice is set up.

If neither is usable, or config.TTS_ENABLED is False, speak() just logs
the text instead of raising -- TTS is explicitly optional per the
integration spec.

speak() blocks until the utterance finishes speaking (Piper: sounddevice
playback + sd.wait(); espeak: subprocess.run, not Popen). That's
deliberate: voice_control_node.py's _speak() wraps this and only resumes
listening once it returns, so a blocking call is what keeps the mic from
ever recording over the robot's own voice.
"""
from __future__ import annotations

import io
import shutil
import subprocess
import sys
import wave
from pathlib import Path
from typing import Optional

from . import config
from .utils import get_logger

_log = get_logger('tts')

_piper_voice = None
_piper_load_attempted = False

_espeak_binary: Optional[str] = None
_espeak_checked = False


def _find_espeak() -> Optional[str]:
    global _espeak_binary, _espeak_checked
    if not _espeak_checked:
        _espeak_binary = shutil.which('espeak-ng') or shutil.which('espeak')
        _espeak_checked = True
        if _espeak_binary is None:
            _log.warning('No espeak-ng/espeak binary found on PATH.')
    return _espeak_binary


def get_piper_voice():
    """Loads and caches the configured Piper voice. Returns None (after
    logging why, once) if Piper isn't usable -- missing package, missing
    model file, or TTS_ENGINE=espeak -- so speak() can fall back."""
    global _piper_voice, _piper_load_attempted
    if _piper_load_attempted:
        return _piper_voice
    _piper_load_attempted = True

    if config.TTS_ENGINE == 'espeak':
        _log.info('TTS_ENGINE=espeak -- skipping Piper.')
        return None

    model_path = config.TTS_VOICE_MODEL
    if not model_path or not Path(model_path).is_file():
        # Logged regardless of TTS_ENGINE (not just when it's forced to
        # "piper") -- a SILENT fallback here is exactly what made a real
        # interpreter/model mismatch look like "nothing I change works":
        # every run just quietly used espeak with no trace of why.
        _log.warning(
            f'No Piper voice model found at {model_path!r} -- falling back to espeak. '
            'Download one from https://github.com/rhasspy/piper/blob/master/VOICES.md'
        )
        return None

    try:
        from piper import PiperVoice
    except ImportError:
        # If you just `pip install piper-tts` and still see this: check
        # it landed in the SAME python3 that actually runs this node.
        # `ros2 run`'s console-script entry point has a hardcoded
        # `#!/usr/bin/python3` shebang -- on a machine with more than one
        # python3 on PATH (pyenv, conda/miniforge, ...) that can silently
        # differ from whichever `python3` a pip install was run under.
        # Check with: /usr/bin/python3 -c "import piper".
        _log.warning(
            f'The "piper-tts" package is not importable from this interpreter ({sys.executable}) '
            '-- falling back to espeak. If you already pip installed it, you likely installed it '
            "into a DIFFERENT python3 than the one actually running this node (ros2 run uses "
            "/usr/bin/python3's shebang, which can differ from an interactively-typed `python3`). "
            f'Install it for the interpreter above, e.g.: {sys.executable} -m pip install --user piper-tts'
        )
        return None

    try:
        _log.info(f'Loading Piper voice from {model_path} ...')
        _piper_voice = PiperVoice.load(model_path)
    except Exception as exc:  # noqa: BLE001 -- any load failure just means "fall back to espeak"
        _log.error(f'Failed to load Piper voice: {exc} -- falling back to espeak.')
        _piper_voice = None
    return _piper_voice


def _speak_piper(text: str) -> bool:
    """Returns True if Piper actually spoke `text`, False if it's
    unavailable/failed and espeak should be tried instead."""
    voice = get_piper_voice()
    if voice is None:
        return False

    try:
        import numpy as np
        import sounddevice as sd

        buf = io.BytesIO()
        with wave.open(buf, 'wb') as wav_file:
            voice.synthesize_wav(text, wav_file)
        buf.seek(0)
        with wave.open(buf, 'rb') as wav_file:
            samplerate = wav_file.getframerate()
            channels = wav_file.getnchannels()
            raw = wav_file.readframes(wav_file.getnframes())
        audio = np.frombuffer(raw, dtype=np.int16)
        if channels > 1:
            audio = audio.reshape(-1, channels)
        sd.play(audio, samplerate)
        sd.wait()
        return True
    except Exception as exc:  # noqa: BLE001 -- playback failure falls back, doesn't crash the node
        _log.error(f'Piper playback failed: {exc} -- falling back to espeak.')
        return False


def _speak_espeak(text: str) -> None:
    binary = _find_espeak()
    if binary is None:
        return
    try:
        subprocess.run([binary, text], check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    except (subprocess.CalledProcessError, OSError) as exc:
        _log.error(f'espeak playback failed: {exc}')


def speak(text: str, enabled: Optional[bool] = None) -> None:
    """Speak `text` out loud (blocking). Always logs the utterance
    regardless of whether audio playback actually happens, so [VOICE]
    logs stay a complete record of what the system said."""
    _log.info(f'[VOICE] "{text}"')

    enabled = config.TTS_ENABLED if enabled is None else enabled
    if not enabled:
        return

    if _speak_piper(text):
        return
    _speak_espeak(text)
