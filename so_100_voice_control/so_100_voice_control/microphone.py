"""Records audio from the input device (in practice, the Bluetooth
mic/earphone already paired as the OS-default input) and saves it as a
WAV file. This is the exact same sounddevice/scipy recording this package
started with -- refactored only into a callable function (record_audio())
so voice_control_node.py can call it directly instead of shelling out to
`python microphone.py`. Running this file directly still works unchanged.

BackgroundListener (added for continuous listening) is a separate,
additive capability: instead of recording a fixed window and always
handing it to Whisper, it keeps the input stream open continuously and
uses a simple energy-based voice-activity detector (VAD) to decide when
someone actually started, and stopped, talking. Only real speech ever
gets queued up as an "utterance" -- silence produces nothing, so nothing
downstream is ever asked to transcribe/guess a command out of dead air.
record_audio() above is untouched and still used by --once/--text testing
and the standalone script.
"""
from __future__ import annotations

import collections
import os
import queue
import tempfile
import threading
from typing import Optional

import numpy as np
import sounddevice as sd
from scipy.io.wavfile import write

from ....so_100_voice_control.so_100_voice_control import config
from ....so_100_voice_control.so_100_voice_control.utils import get_logger

_log = get_logger('microphone')

DEFAULT_OUTPUT_PATH = os.path.join(tempfile.gettempdir(), 'bluetooth_mic.wav')


def record_audio(
    duration: Optional[float] = None,
    samplerate: Optional[int] = None,
    device=None,
    output_path: Optional[str] = None,
) -> str:
    """Record `duration` seconds of mono audio and write it to a WAV file.

    Any argument left as None falls back to config.py (env-var
    configurable) so callers don't need to know the defaults. `device`
    accepts whatever sounddevice accepts (index or name substring) --
    passing None keeps using the OS-default input device, same as the
    original script.

    Returns the path the WAV file was written to.
    """
    duration = config.RECORD_DURATION if duration is None else duration
    samplerate = config.SAMPLE_RATE if samplerate is None else samplerate
    device = config.MIC_DEVICE if device is None else device
    output_path = output_path or DEFAULT_OUTPUT_PATH

    print('Recording...')
    audio = sd.rec(
        int(duration * samplerate),
        samplerate=samplerate,
        channels=1,
        dtype='int16',
        device=device,
    )
    sd.wait()

    write(output_path, samplerate, audio)
    print(f'Saved! ({output_path})')
    return output_path


def save_wav(audio: np.ndarray, samplerate: Optional[int] = None, output_path: Optional[str] = None) -> str:
    """Write an int16 mono numpy array (as produced by BackgroundListener)
    to a WAV file, same format record_audio() writes. Returns the path."""
    samplerate = config.SAMPLE_RATE if samplerate is None else samplerate
    output_path = output_path or DEFAULT_OUTPUT_PATH
    write(output_path, samplerate, audio)
    return output_path


class BackgroundListener:
    """Keeps the input device open continuously and hands back one int16
    mono numpy array per detected utterance via get_utterance().

    State machine (per analysis frame, VAD_FRAME_MS each):
        IDLE     -- frame RMS below threshold: discarded, except kept in a
                    short rolling pre-roll buffer in case speech starts on
                    the very next frame (so the first word isn't clipped).
        SPEAKING -- triggered once a frame's RMS crosses VAD_ENERGY_THRESHOLD.
                    Frames keep accumulating (including brief pauses)
                    until VAD_SILENCE_DURATION_SEC of continuous silence
                    is seen, or VAD_MAX_UTTERANCE_DURATION_SEC is hit.
                    On end, the accumulated audio is queued as one
                    utterance UNLESS it's shorter than
                    VAD_MIN_SPEECH_DURATION_SEC (a cough, a click, a door
                    closing) -- those are silently discarded.

    All of this runs inside sounddevice's audio callback thread, which
    must never block -- so the callback only does cheap numpy math and
    queue.Queue.put(); transcription happens on the caller's thread via
    get_utterance().
    """

    def __init__(
        self,
        samplerate: Optional[int] = None,
        device=None,
        energy_threshold: Optional[float] = None,
        frame_ms: Optional[int] = None,
        silence_duration_sec: Optional[float] = None,
        min_speech_duration_sec: Optional[float] = None,
        max_utterance_duration_sec: Optional[float] = None,
        pre_roll_ms: Optional[int] = None,
    ) -> None:
        self._samplerate = config.SAMPLE_RATE if samplerate is None else samplerate
        self._device = config.MIC_DEVICE if device is None else device
        self._energy_threshold = config.VAD_ENERGY_THRESHOLD if energy_threshold is None else energy_threshold
        frame_ms = config.VAD_FRAME_MS if frame_ms is None else frame_ms
        silence_duration_sec = config.VAD_SILENCE_DURATION_SEC if silence_duration_sec is None else silence_duration_sec
        min_speech_duration_sec = config.VAD_MIN_SPEECH_DURATION_SEC if min_speech_duration_sec is None else min_speech_duration_sec
        max_utterance_duration_sec = (
            config.VAD_MAX_UTTERANCE_DURATION_SEC if max_utterance_duration_sec is None else max_utterance_duration_sec
        )
        pre_roll_ms = config.VAD_PRE_ROLL_MS if pre_roll_ms is None else pre_roll_ms

        self._frame_samples = max(1, int(self._samplerate * frame_ms / 1000))
        self._silence_frames_needed = max(1, int(silence_duration_sec * 1000 / frame_ms))
        self._min_speech_frames = max(1, int(min_speech_duration_sec * 1000 / frame_ms))
        self._max_frames = max(1, int(max_utterance_duration_sec * 1000 / frame_ms))
        pre_roll_frames = max(1, int(pre_roll_ms / frame_ms))

        self._pre_roll: collections.deque = collections.deque(maxlen=pre_roll_frames)
        self._queue: "queue.Queue[np.ndarray]" = queue.Queue()
        self._paused = threading.Event()
        self._speaking = False
        self._speech_frames: list = []
        self._speech_frame_count = 0  # frames that were ACTUALLY speech, not pre-roll/trailing silence padding
        self._silence_run = 0
        self._stream: Optional[sd.InputStream] = None

    def start(self) -> None:
        self._stream = sd.InputStream(
            samplerate=self._samplerate,
            channels=1,
            dtype='int16',
            device=self._device,
            blocksize=self._frame_samples,
            callback=self._on_audio,
        )
        self._stream.start()
        _log.info('[VOICE] Listening in the background -- just speak, no need to wait for a prompt.')

    def stop(self) -> None:
        if self._stream is not None:
            self._stream.stop()
            self._stream.close()
            self._stream = None

    def pause(self) -> None:
        """Stop turning incoming audio into utterances (called while TTS
        is speaking, so the robot never transcribes/reacts to its own
        voice coming back through the mic). Any in-progress utterance is
        dropped rather than paused mid-word -- resuming mid-utterance
        would just glue unrelated audio together."""
        self._paused.set()
        self._speaking = False
        self._speech_frames = []
        self._speech_frame_count = 0
        self._silence_run = 0

    def resume(self) -> None:
        self._paused.clear()

    def get_utterance(self, timeout: Optional[float] = None) -> Optional[np.ndarray]:
        """Block up to `timeout` seconds for the next detected utterance.
        Returns None on timeout -- callers should just loop and check
        again, not treat it as an error or as "no command given"."""
        try:
            return self._queue.get(timeout=timeout)
        except queue.Empty:
            return None

    def _on_audio(self, indata, frames, time_info, status) -> None:  # noqa: ARG002 -- sounddevice callback signature
        if status:
            _log.warning(f'Audio input status: {status}')
        if self._paused.is_set():
            return

        chunk = indata[:, 0].copy()
        rms = float(np.sqrt(np.mean(chunk.astype(np.float64) ** 2)))
        is_speech = rms > self._energy_threshold

        if not self._speaking:
            self._pre_roll.append(chunk)
            if is_speech:
                self._speaking = True
                self._speech_frames = list(self._pre_roll)
                self._speech_frame_count = 1  # the triggering frame itself
                self._silence_run = 0
            return

        self._speech_frames.append(chunk)
        if is_speech:
            self._speech_frame_count += 1
            self._silence_run = 0
        else:
            self._silence_run += 1

        finished_by_silence = self._silence_run >= self._silence_frames_needed
        finished_by_length = len(self._speech_frames) >= self._max_frames
        if finished_by_silence or finished_by_length:
            # Gate on how many frames were ACTUAL speech, not the total
            # buffer length -- the buffer always includes pre-roll and the
            # trailing silence padding waited out above, so a length check
            # against len(self._speech_frames) would never reject a short
            # blip once that padding alone exceeds min_speech_frames.
            if self._speech_frame_count >= self._min_speech_frames:
                self._queue.put(np.concatenate(self._speech_frames))
            self._speaking = False
            self._speech_frames = []
            self._speech_frame_count = 0
            self._silence_run = 0
            self._pre_roll.clear()


if __name__ == '__main__':
    # Preserves the original standalone behavior: `python microphone.py`
    # records into ./bluetooth_mic.wav in the current directory.
    record_audio(output_path='bluetooth_mic.wav')
