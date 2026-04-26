"""Detect traffic lights in a frame using YOLOv8 pretrained on COCO."""
from dataclasses import dataclass, field
from typing import List, Optional, Sequence, Tuple

import numpy as np


# Bounding box format used everywhere here: (x1, y1, x2, y2) in pixels.
BBox = Tuple[int, int, int, int]


@dataclass
class Detection:
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
    def aspect_ratio(self) -> float:
        """Height-over-width. Vertical objects (traffic lights) are > 1."""
        if self.width <= 0:
            return 0.0
        return self.height / self.width

    @property
    def center(self) -> Tuple[int, int]:
        return ((self.x1 + self.x2) // 2, (self.y1 + self.y2) // 2)

    def crop(self, image: np.ndarray) -> np.ndarray:
        h, w = image.shape[:2]
        x1 = max(0, self.x1)
        y1 = max(0, self.y1)
        x2 = min(w, self.x2)
        y2 = min(h, self.y2)
        return image[y1:y2, x1:x2]

    def overlaps_region(self, region: BBox, min_iou: float = 0.0) -> bool:
        """True if this detection overlaps the given region.

        With min_iou=0 (default), any pixel of overlap counts. Use a positive
        min_iou to require a more substantial overlap.
        """
        rx1, ry1, rx2, ry2 = region
        ix1 = max(self.x1, rx1)
        iy1 = max(self.y1, ry1)
        ix2 = min(self.x2, rx2)
        iy2 = min(self.y2, ry2)
        if ix2 <= ix1 or iy2 <= iy1:
            return False
        if min_iou <= 0:
            return True
        inter = (ix2 - ix1) * (iy2 - iy1)
        union = self.area + (rx2 - rx1) * (ry2 - ry1) - inter
        return (inter / union) >= min_iou


class YOLOTrafficLightDetector:
    """Detect traffic lights with YOLOv8 (pretrained on COCO).

    COCO class id 9 is `traffic light`. The detector also queries class 11
    (`stop sign`) and discards any traffic-light hit that overlaps a stop-sign
    hit — this kills the most common confusion (round red signs like 'no
    entry' that visually resemble a lit red bulb).

    Post-filters applied to surviving detections:
      - confidence threshold
      - minimum bounding-box size (kills tiny noise)
      - aspect ratio range (traffic lights are vertical and narrow)
      - ignore regions (e.g. mask out fixed UI overlays in test imagery)
    """

    _COCO_TRAFFIC_LIGHT_CLASS = 9
    _COCO_STOP_SIGN_CLASS = 11
    _SIGN_OVERLAP_IOU = 0.20

    def __init__(
        self,
        model_size: str = "n",
        min_confidence: float = 0.30,
        min_aspect_ratio: float = 1.4,
        max_aspect_ratio: float = 5.0,
        min_size_px: int = 12,
        ignore_regions: Optional[Sequence[BBox]] = None,
        imgsz: int = 1280,
        weights: Optional[str] = None,
    ):
        from ultralytics import YOLO

        weights = weights or f"yolov8{model_size}.pt"
        self._model = YOLO(weights)
        self.min_confidence = min_confidence
        self.min_aspect_ratio = min_aspect_ratio
        self.max_aspect_ratio = max_aspect_ratio
        self.min_size_px = min_size_px
        self.ignore_regions: List[BBox] = list(ignore_regions or [])
        self.imgsz = imgsz

    def _passes_filters(self, d: Detection) -> bool:
        if d.width < self.min_size_px or d.height < self.min_size_px:
            return False
        ar = d.aspect_ratio
        if ar < self.min_aspect_ratio or ar > self.max_aspect_ratio:
            return False
        for region in self.ignore_regions:
            if d.overlaps_region(region):
                return False
        return True

    def detect(self, image: np.ndarray) -> List[Detection]:
        """Run detection on a BGR image. Returns filtered traffic-light hits."""
        results = self._model(
            image,
            classes=[self._COCO_TRAFFIC_LIGHT_CLASS, self._COCO_STOP_SIGN_CLASS],
            conf=self.min_confidence,
            imgsz=self.imgsz,
            verbose=False,
        )

        traffic_lights: List[Detection] = []
        sign_regions: List[BBox] = []
        for r in results:
            if r.boxes is None:
                continue
            for box in r.boxes:
                xyxy = box.xyxy[0].cpu().numpy()
                cls_id = int(box.cls[0])
                d = Detection(
                    x1=int(xyxy[0]),
                    y1=int(xyxy[1]),
                    x2=int(xyxy[2]),
                    y2=int(xyxy[3]),
                    confidence=float(box.conf[0]),
                )
                if cls_id == self._COCO_TRAFFIC_LIGHT_CLASS:
                    traffic_lights.append(d)
                elif cls_id == self._COCO_STOP_SIGN_CLASS:
                    sign_regions.append((d.x1, d.y1, d.x2, d.y2))

        out: List[Detection] = []
        for d in traffic_lights:
            if not self._passes_filters(d):
                continue
            if any(d.overlaps_region(r, min_iou=self._SIGN_OVERLAP_IOU) for r in sign_regions):
                continue
            out.append(d)
        return out
