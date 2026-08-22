"""A rolling, time-windowed buffer: old points fall off automatically once
they're older than `history_seconds`, so telemetry never grows unbounded
in memory no matter how long the dashboard runs.
"""
from __future__ import annotations

from collections import deque
from typing import Tuple

import numpy as np


class RollingBuffer:
    def __init__(self, history_seconds: float, max_points: int = 20000) -> None:
        self._history_seconds = history_seconds
        self._times: "deque[float]" = deque(maxlen=max_points)
        self._values: "deque[float]" = deque(maxlen=max_points)

    def append(self, t: float, v: float) -> None:
        self._times.append(t)
        self._values.append(v)
        self._trim(t)

    def _trim(self, now: float) -> None:
        cutoff = now - self._history_seconds
        while self._times and self._times[0] < cutoff:
            self._times.popleft()
            self._values.popleft()

    def set_history_seconds(self, seconds: float) -> None:
        self._history_seconds = seconds
        if self._times:
            self._trim(self._times[-1])

    def clear(self) -> None:
        self._times.clear()
        self._values.clear()

    def as_arrays(self) -> Tuple[np.ndarray, np.ndarray]:
        if not self._times:
            return np.empty(0), np.empty(0)
        return np.fromiter(self._times, dtype=float), np.fromiter(self._values, dtype=float)

    def __len__(self) -> int:
        return len(self._times)
