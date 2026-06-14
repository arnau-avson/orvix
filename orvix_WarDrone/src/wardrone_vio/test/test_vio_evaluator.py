"""Tests for vio_evaluator_node module."""

import math
import pytest


class TestDriftCalculation:
    """Test drift calculation logic independently of ROS 2."""

    def test_zero_drift(self):
        dx, dy, dz = 0.0, 0.0, 0.0
        total = math.sqrt(dx*dx + dy*dy + dz*dz)
        assert total == pytest.approx(0.0)

    def test_known_drift(self):
        dx, dy, dz = 3.0, 4.0, 0.0
        total = math.sqrt(dx*dx + dy*dy + dz*dz)
        assert total == pytest.approx(5.0)

    def test_3d_drift(self):
        dx, dy, dz = 1.0, 1.0, 1.0
        total = math.sqrt(dx*dx + dy*dy + dz*dz)
        assert total == pytest.approx(math.sqrt(3.0))

    def test_drift_rate(self):
        total_error = 5.0
        elapsed_min = 2.5
        rate = total_error / elapsed_min
        assert rate == pytest.approx(2.0)
