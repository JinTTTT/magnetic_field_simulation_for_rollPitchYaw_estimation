"""Thread-safe rolling average for timestamped magnetic field vectors."""

from collections import deque
from dataclasses import dataclass
import threading

import numpy as np


@dataclass(frozen=True)
class RollingAverageSnapshot:
    mean_mT: np.ndarray
    start_time: float
    end_time: float
    sample_count: int
    window_size: int
    version: int

    @property
    def ready(self):
        return self.sample_count == self.window_size


class RollingFieldAverage:
    """Maintain an O(1) rolling mean and expose consistent snapshots."""

    def __init__(self, window_size, channel_count=6):
        if window_size < 1:
            raise ValueError("window_size must be at least 1")
        if channel_count < 1:
            raise ValueError("channel_count must be at least 1")
        self.window_size = int(window_size)
        self.channel_count = int(channel_count)
        self._samples = deque()
        self._sum = np.zeros(self.channel_count, dtype=float)
        self._version = 0
        self._closed = False
        self._condition = threading.Condition()

    def append(self, values_mT, start_time, end_time):
        values = np.asarray(values_mT, dtype=float)
        if values.shape != (self.channel_count,) or not np.isfinite(values).all():
            raise ValueError(
                f"field sample must contain {self.channel_count} finite channels"
            )
        start = float(start_time)
        end = float(end_time)
        if not np.isfinite((start, end)).all() or end < start:
            raise ValueError("sample timestamps must be finite and ordered")

        with self._condition:
            if self._closed:
                raise RuntimeError("cannot append to a closed rolling average")
            if self._samples and start < self._samples[-1][1]:
                raise ValueError("sample timestamps must increase monotonically")
            if len(self._samples) == self.window_size:
                _old_start, _old_end, old_values = self._samples.popleft()
                self._sum -= old_values
            stored = values.copy()
            self._samples.append((start, end, stored))
            self._sum += stored
            self._version += 1
            self._condition.notify_all()

    def wait_for_snapshot(self, after_version=0, timeout=None):
        """Wait for a newer sample and return one internally consistent view."""
        with self._condition:
            changed = self._condition.wait_for(
                lambda: self._version > after_version or self._closed,
                timeout=timeout,
            )
            if not changed or not self._samples:
                return None
            return self._snapshot_locked()

    def snapshot(self):
        with self._condition:
            return None if not self._samples else self._snapshot_locked()

    def close(self):
        with self._condition:
            self._closed = True
            self._condition.notify_all()

    def _snapshot_locked(self):
        return RollingAverageSnapshot(
            mean_mT=(self._sum / len(self._samples)).copy(),
            start_time=self._samples[0][0],
            end_time=self._samples[-1][1],
            sample_count=len(self._samples),
            window_size=self.window_size,
            version=self._version,
        )
