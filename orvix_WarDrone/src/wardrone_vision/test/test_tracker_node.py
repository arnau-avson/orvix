"""Tests for tracker_node module."""

import pytest
from wardrone_vision.tracker_node import PIDController


class TestPIDController:
    def test_proportional(self):
        pid = PIDController(kp=1.0, ki=0.0, kd=0.0)
        output = pid.compute(0.5, dt=0.1)
        assert output == pytest.approx(0.5)

    def test_clamping(self):
        pid = PIDController(kp=10.0, output_min=-1.0, output_max=1.0)
        output = pid.compute(5.0, dt=0.1)
        assert output == pytest.approx(1.0)

    def test_negative_clamping(self):
        pid = PIDController(kp=10.0, output_min=-1.0, output_max=1.0)
        output = pid.compute(-5.0, dt=0.1)
        assert output == pytest.approx(-1.0)

    def test_zero_error(self):
        pid = PIDController(kp=1.0, ki=0.0, kd=0.0)
        output = pid.compute(0.0, dt=0.1)
        assert output == pytest.approx(0.0)

    def test_integral(self):
        pid = PIDController(kp=0.0, ki=1.0, kd=0.0, output_min=-10.0, output_max=10.0)
        pid.compute(1.0, dt=0.1)  # integral = 0.1
        output = pid.compute(1.0, dt=0.1)  # integral = 0.2
        assert output == pytest.approx(0.2)

    def test_reset(self):
        pid = PIDController(kp=0.0, ki=1.0, kd=0.0, output_min=-10.0, output_max=10.0)
        pid.compute(1.0, dt=0.1)
        pid.reset()
        output = pid.compute(1.0, dt=0.1)
        assert output == pytest.approx(0.1)  # Fresh start


class TestTrackerNodeImport:
    def test_import(self):
        from wardrone_vision.tracker_node import TrackerNode
        assert TrackerNode is not None
