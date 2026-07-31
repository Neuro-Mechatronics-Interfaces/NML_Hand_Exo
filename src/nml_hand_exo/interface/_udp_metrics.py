"""Qt-free metrics used by the UDP receiver and GUI display."""

from __future__ import annotations

import math
import time


class TimeWeightedBacklogEMA:
    """Track current queue depth and a time-weighted exponential average."""

    def __init__(self, time_constant_s: float = 2.0):
        if time_constant_s <= 0:
            raise ValueError("time_constant_s must be positive")
        self.time_constant_s = float(time_constant_s)
        self.current = 0
        self.ema = 0.0
        self._last_update = time.monotonic()

    def reset(self, now: float | None = None) -> None:
        self.current = 0
        self.ema = 0.0
        self._last_update = time.monotonic() if now is None else float(now)

    def enqueue(self, count: int = 1, now: float | None = None) -> None:
        self._advance(now)
        self.current += max(0, int(count))

    def complete(self, count: int = 1, now: float | None = None) -> None:
        self._advance(now)
        self.current = max(0, self.current - max(0, int(count)))

    def snapshot(self, now: float | None = None) -> tuple[int, float]:
        self._advance(now)
        return self.current, self.ema

    def _advance(self, now: float | None) -> None:
        current_time = time.monotonic() if now is None else float(now)
        elapsed = max(0.0, current_time - self._last_update)
        if elapsed:
            retained = math.exp(-elapsed / self.time_constant_s)
            self.ema = retained * self.ema + (1.0 - retained) * self.current
        self._last_update = current_time
