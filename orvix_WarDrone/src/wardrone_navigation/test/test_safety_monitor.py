"""Tests for safety_monitor_node module."""

import pytest


class TestSafetyMonitorImport:
    def test_import(self):
        from wardrone_navigation.safety_monitor_node import SafetyMonitorNode
        assert SafetyMonitorNode is not None
