"""Obstacle detection for a sidewalk delivery robot.

Reuses the same YOLOv8 backbone as the traffic-light detector but queries
COCO classes relevant for sidewalk navigation: pedestrians, bikes, vehicles,
animals, street furniture. Returns rich `Obstacle` records the navigation
layer can use to decide whether to stop, slow, or yield.
"""
from dataclasses import dataclass
from typing import List, Optional, Tuple

import numpy as np


# COCO class id → friendly name. Curated for sidewalk navigation: things
# the robot needs to react to (people, vehicles, animals, street furniture
# that could be in its path). Anything not here is ignored at the source.
_OBSTACLE_CLASSES = {
    0: "person",
    1: "bicycle",
    2: "car",
    3: "motorcycle",
    5: "bus",
    7: "truck",
    13: "bench",
    16: "dog",
    17: "cat",
    18: "horse",
    24: "backpack",
    28: "suitcase",
    56: "chair",
    57: "couch",
    58: "potted_plant",
}

# Per-class severity. Higher = more urgent to stop for. Used by `should_stop`
# to decide which obstacle dominates when several are visible.
_SEVERITY = {
    "person": 5,
    "dog": 5,
    "cat": 4,
    "bicycle": 4,
    "motorcycle": 5,
    "car": 5,
    "bus": 5,
    "truck": 5,
    "horse": 5,
    "backpack": 1,
    "suitcase": 1,
    "bench": 2,
    "chair": 2,
    "couch": 2,
    "potted_plant": 2,
}


@dataclass
class Obstacle:
    class_name: str
    cls_id: int
    x1: int
    y1: int
    x2: int
    y2: int
    confidence: float

    @property
    def width(self) -> int:
        return self.x2 - self.x1

    @property
    def height(self) -> int:
        return self.y2 - self.y1

    @property
    def area(self) -> int:
        return max(0, self.width) * max(0, self.height)

    @property
    def center_x(self) -> int:
        return (self.x1 + self.x2) // 2

    @property
    def center_y(self) -> int:
        return (self.y1 + self.y2) // 2

    @property
    def severity(self) -> int:
        return _SEVERITY.get(self.class_name, 1)

    def crop(self, image: np.ndarray) -> np.ndarray:
        h, w = image.shape[:2]
        return image[max(0, self.y1):min(h, self.y2), max(0, self.x1):min(w, self.x2)]


class ObstacleDetector:
    """YOLOv8-based obstacle detector restricted to navigation-relevant classes."""

    def __init__(
        self,
        model_size: str = "m",
        min_confidence: float = 0.30,
        imgsz: int = 1280,
        weights: Optional[str] = None,
    ):
        from ultralytics import YOLO
        weights = weights or f"yolov8{model_size}.pt"
        self._model = YOLO(weights)
        self.min_confidence = min_confidence
        self.imgsz = imgsz
        self._class_ids = list(_OBSTACLE_CLASSES.keys())

    def detect(self, image: np.ndarray) -> List[Obstacle]:
        results = self._model(
            image,
            classes=self._class_ids,
            conf=self.min_confidence,
            imgsz=self.imgsz,
            verbose=False,
        )

        out: List[Obstacle] = []
        for r in results:
            if r.boxes is None:
                continue
            for box in r.boxes:
                xyxy = box.xyxy[0].cpu().numpy()
                cls_id = int(box.cls[0])
                out.append(
                    Obstacle(
                        class_name=_OBSTACLE_CLASSES.get(cls_id, str(cls_id)),
                        cls_id=cls_id,
                        x1=int(xyxy[0]),
                        y1=int(xyxy[1]),
                        x2=int(xyxy[2]),
                        y2=int(xyxy[3]),
                        confidence=float(box.conf[0]),
                    )
                )
        return out


def should_stop(
    obstacles: List[Obstacle],
    frame_shape: Tuple[int, ...],
    close_height_ratio: float = 0.30,
    center_band: Tuple[float, float] = (0.30, 0.70),
    min_severity: int = 3,
) -> Optional[Obstacle]:
    """Return the obstacle that justifies stopping, or None.

    Decision: stop if any obstacle is BIG (height ≥ `close_height_ratio` of
    the frame) AND its center sits in the central horizontal band (i.e. in
    the robot's path) AND its severity meets the threshold.

    Bbox height is a proxy for proximity. A more sophisticated version would
    use camera intrinsics + object real-world dimensions to estimate metric
    distance; here we use pixel ratio because the robot's camera intrinsics
    aren't fixed yet.

    Returns the most severe blocking obstacle (so the navigation layer can
    log *why* it stopped), or None when the path is clear.
    """
    h, w = frame_shape[:2]
    cx_min, cx_max = center_band[0] * w, center_band[1] * w

    blockers: List[Obstacle] = []
    for o in obstacles:
        if o.severity < min_severity:
            continue
        if o.height / h < close_height_ratio:
            continue
        if not (cx_min <= o.center_x <= cx_max):
            continue
        blockers.append(o)

    if not blockers:
        return None
    return max(blockers, key=lambda o: (o.severity, o.area))
