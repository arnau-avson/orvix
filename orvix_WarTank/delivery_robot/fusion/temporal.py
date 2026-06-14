"""Sliding-window majority voting on a stream of categorical observations.

Used to suppress single-frame perception flicker — a one-frame misclassification
shouldn't trigger a real-world action like crossing a road.
"""
from collections import Counter, deque
from typing import Deque, Optional


class TemporalStateVoter:
    def __init__(self, window_size: int = 5, min_agreement: int = 3):
        if min_agreement > window_size:
            raise ValueError("min_agreement cannot exceed window_size")
        self.window_size = window_size
        self.min_agreement = min_agreement
        self._window: Deque[str] = deque(maxlen=window_size)

    def observe(self, state: str) -> None:
        self._window.append(state)

    def fused(self, fallback: str = "unknown") -> str:
        """Return the majority state if it has reached `min_agreement`,
        otherwise the fallback. Empty window also returns the fallback.
        """
        if not self._window:
            return fallback
        most_common, count = Counter(self._window).most_common(1)[0]
        if count >= self.min_agreement:
            return most_common
        return fallback

    @property
    def filled(self) -> bool:
        return len(self._window) >= self.window_size

    @property
    def latest(self) -> Optional[str]:
        return self._window[-1] if self._window else None

    def reset(self) -> None:
        self._window.clear()
