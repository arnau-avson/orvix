"""Tests for detector_node module."""

import pytest


class TestDetectorNodeImport:
    def test_import(self):
        from wardrone_vision.detector_node import DetectorNode
        assert DetectorNode is not None
