"""YOLO inference wrapper.

Provides a clean interface to ultralytics YOLO for object detection.
Designed to be testable independently of ROS 2.
"""

from dataclasses import dataclass
from typing import List, Optional
import numpy as np


@dataclass
class DetectionResult:
    """A single detection from YOLO."""
    class_id: int
    class_name: str
    confidence: float
    x1: int
    y1: int
    x2: int
    y2: int

    @property
    def cx(self) -> int:
        return (self.x1 + self.x2) // 2

    @property
    def cy(self) -> int:
        return (self.y1 + self.y2) // 2

    @property
    def width(self) -> int:
        return self.x2 - self.x1

    @property
    def height(self) -> int:
        return self.y2 - self.y1

    @property
    def area(self) -> int:
        return self.width * self.height


class YoloWrapper:
    """Wrapper around ultralytics YOLO model for inference."""

    # COCO class names (subset)
    COCO_NAMES = {
        0: 'person', 1: 'bicycle', 2: 'car', 3: 'motorcycle',
        5: 'bus', 6: 'train', 7: 'truck', 14: 'bird', 15: 'cat',
        16: 'dog', 24: 'backpack', 25: 'umbrella', 39: 'bottle',
        56: 'chair', 57: 'couch', 59: 'bed', 60: 'dining table',
        62: 'tv', 63: 'laptop', 67: 'cell phone',
    }

    def __init__(
        self,
        model_path: str = 'yolo11n.pt',
        confidence_threshold: float = 0.5,
        target_classes: Optional[List[int]] = None,
        inference_size: int = 640,
        device: str = 'cpu',
    ):
        self._model_path = model_path
        self._confidence_threshold = confidence_threshold
        self._target_classes = target_classes or [0, 2, 5, 7]
        self._inference_size = inference_size
        self._device = device
        self._model = None

    def load_model(self):
        """Load the YOLO model. Call once before inference."""
        from ultralytics import YOLO
        self._model = YOLO(self._model_path)
        if self._device != 'cpu':
            self._model.to(self._device)

    def detect(self, frame: np.ndarray) -> List[DetectionResult]:
        """Run YOLO inference on a single frame.

        Args:
            frame: BGR image as numpy array (H, W, 3).

        Returns:
            List of DetectionResult for detections matching target classes
            and exceeding confidence threshold.
        """
        if self._model is None:
            raise RuntimeError("Model not loaded. Call load_model() first.")

        results = self._model(
            frame,
            imgsz=self._inference_size,
            conf=self._confidence_threshold,
            classes=self._target_classes,
            verbose=False,
        )

        detections = []
        for result in results:
            if result.boxes is None:
                continue
            for box in result.boxes:
                class_id = int(box.cls[0])
                confidence = float(box.conf[0])
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)

                class_name = self.COCO_NAMES.get(class_id, f'class_{class_id}')

                detections.append(DetectionResult(
                    class_id=class_id,
                    class_name=class_name,
                    confidence=confidence,
                    x1=int(x1),
                    y1=int(y1),
                    x2=int(x2),
                    y2=int(y2),
                ))

        return detections

    @staticmethod
    def draw_detections(frame: np.ndarray, detections: List[DetectionResult]) -> np.ndarray:
        """Draw bounding boxes on the frame for visualization."""
        import cv2
        annotated = frame.copy()
        for det in detections:
            color = (0, 255, 0)
            cv2.rectangle(annotated, (det.x1, det.y1), (det.x2, det.y2), color, 2)
            label = f'{det.class_name} {det.confidence:.2f}'
            cv2.putText(annotated, label, (det.x1, det.y1 - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
        return annotated
