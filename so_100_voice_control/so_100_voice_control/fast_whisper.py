"""Transcribes recorded speech with faster-whisper. Same model/backend the
package started with (WhisperModel, cpu, int8) -- refactored only so the
model loads ONCE and is reused across calls (get_model() caches it at
module scope) instead of reloading on every transcription, and so
transcribe() is a plain function other modules can call directly instead
of this file only working as `python fast_whisper.py`. Running this file
directly still works unchanged (records 5s live, then transcribes).
"""
from __future__ import annotations

from typing import Optional, Union

import numpy as np
from faster_whisper import WhisperModel

from ....so_100_voice_control.so_100_voice_control import config

_model: Optional[WhisperModel] = None


def get_model() -> WhisperModel:
    """Loads the WhisperModel on first call and reuses it after that --
    loading it fresh per-transcription (as the original script did when
    run repeatedly) is the latency mistake the integration explicitly
    needs to avoid."""
    global _model
    if _model is None:
        print(f'Loading Whisper model "{config.WHISPER_MODEL}" ({config.WHISPER_DEVICE}/{config.WHISPER_COMPUTE_TYPE})...')
        _model = WhisperModel(
            config.WHISPER_MODEL,
            device=config.WHISPER_DEVICE,
            compute_type=config.WHISPER_COMPUTE_TYPE,
        )
    return _model


def transcribe(audio: Union[str, np.ndarray], language: Optional[str] = None) -> str:
    """Transcribe a WAV file path (e.g. from microphone.record_audio()) or
    a raw float32 mono numpy array, and return the plain-text result.
    """
    model = get_model()
    language = language or config.WHISPER_LANGUAGE
    segments, _info = model.transcribe(audio, language=language)

    text = ''
    for segment in segments:
        text += segment.text
    return text.strip()


if __name__ == '__main__':
    import sounddevice as sd

    print('Speak now...')
    audio = sd.rec(
        int(config.RECORD_DURATION * config.SAMPLE_RATE),
        samplerate=config.SAMPLE_RATE,
        channels=1,
        dtype='float32',
    )
    sd.wait()
    print('Recording finished.')
    print('Transcribing...')

    text = transcribe(audio.flatten())

    print('\nYou said:')
    print(text)
