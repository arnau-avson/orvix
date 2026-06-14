"""Tests for yolo_wrapper module."""

import pytest
import numpy as np
from wardrone_vision.yolo_wrapper import DetectionResult, YoloWrapper


class TestDetectionResult:
    def test_properties(self):
        d = DetectionResult(class_id=2, class_name='car', confidence=0.9,
                           x1=100, y1=200, x2=300, y2=400)
        assert d.cx == 200
        assert d.cy == 300
        assert d.width == 200
        assert d.height == 200
        assert d.area == 40000

    def test_zero_size(self):
        d = DetectionResult(class_id=0, class_name='person', confidence=0.5,
                           x1=100, y1=100, x2=100, y2=100)
        assert d.width == 0
        assert d.height == 0
        assert d.area == 0


class TestYoloWrapper:
    def test_coco_names(self):
        assert YoloWrapper.COCO_NAMES[0] == 'person'
        assert YoloWrapper.COCO_NAMES[2] == 'car'
        assert YoloWrapper.COCO_NAMES[7] == 'truck'

    def test_init_defaults(self):
        wrapper = YoloWrapper()
        assert wrapper._confidence_threshold == 0.5
        assert wrapper._device == 'cpu'
        assert 0 in wrapper._target_classes

    def test_detect_without_model_raises(self):
        wrapper = YoloWrapper()
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        with pytest.raises(RuntimeError, match="not loaded"):
            wrapper.detect(frame)
