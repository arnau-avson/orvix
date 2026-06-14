"""Camera input adapters.

`CameraSource.read()` returns a BGR `numpy.ndarray` (the format OpenCV and
the perception layer expect). All sources are usable as context managers.

Concrete implementations:
- `OpenCVCamera`: wraps `cv2.VideoCapture`. Works for USB webcams (integer
  index), local video files (path), and IP cameras (RTSP/HTTP URL) — same
  call shape, OpenCV picks the right backend.
- `ImageSequenceCamera`: cycles through a list of still images at a target
  fps. Use for testing the loop without a real camera.
- `BlankCamera`: always returns an all-black frame. Use for unit tests when
  perception output should be deterministic.
"""
import time
from abc import ABC, abstractmethod
from pathlib import Path
from typing import List, Optional, Sequence, Union

import cv2
import numpy as np


class CameraSource(ABC):
    @abstractmethod
    def read(self) -> Optional[np.ndarray]:
        """Return next BGR frame, or None if the source is exhausted/errored."""

    @abstractmethod
    def close(self) -> None:
        """Release any underlying resources (file handles, sockets)."""

    def __enter__(self) -> "CameraSource":
        return self

    def __exit__(self, *exc_info) -> None:
        self.close()


class OpenCVCamera(CameraSource):
    """USB webcam (int index), video file (path), or IP cam (RTSP/HTTP URL)."""

    def __init__(
        self,
        source: Union[int, str],
        width: Optional[int] = None,
        height: Optional[int] = None,
        target_fps: Optional[float] = None,
    ):
        self._cap = cv2.VideoCapture(source)
        if not self._cap.isOpened():
            raise RuntimeError(f"Cannot open camera source: {source!r}")
        if width is not None:
            self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, int(width))
        if height is not None:
            self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, int(height))
        if target_fps is not None:
            self._cap.set(cv2.CAP_PROP_FPS, float(target_fps))

    def read(self) -> Optional[np.ndarray]:
        ok, frame = self._cap.read()
        return frame if ok else None

    def close(self) -> None:
        self._cap.release()


class ImageSequenceCamera(CameraSource):
    """Plays a fixed list of images. Throttles to `fps` so the perception
    pipeline sees a realistic frame cadence rather than blowing through them.
    """

    def __init__(
        self,
        image_paths: Sequence[Union[str, Path]],
        loop: bool = True,
        fps: float = 10.0,
    ):
        self._frames: List[np.ndarray] = []
        for p in image_paths:
            frame = cv2.imread(str(p))
            if frame is not None:
                self._frames.append(frame)
        if not self._frames:
            raise ValueError("No readable images in the sequence.")
        self._idx = 0
        self._loop = loop
        self._frame_period_s = 1.0 / max(fps, 0.01)
        self._next_frame_at = time.monotonic()

    def read(self) -> Optional[np.ndarray]:
        if self._idx >= len(self._frames):
            if not self._loop:
                return None
            self._idx = 0
        wait = self._next_frame_at - time.monotonic()
        if wait > 0:
            time.sleep(wait)
        frame = self._frames[self._idx]
        self._idx += 1
        self._next_frame_at = time.monotonic() + self._frame_period_s
        return frame

    def close(self) -> None:
        return None


class BlankCamera(CameraSource):
    def __init__(self, width: int = 640, height: int = 480):
        self._frame = np.zeros((height, width, 3), dtype=np.uint8)

    def read(self) -> Optional[np.ndarray]:
        return self._frame.copy()

    def close(self) -> None:
        return None
