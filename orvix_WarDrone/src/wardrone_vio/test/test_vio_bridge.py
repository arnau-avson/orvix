"""Tests for vio_bridge_node module."""

import pytest


class TestVioBridgeImport:
    def test_import(self):
        from wardrone_vio.vio_bridge_node import VioBridgeNode
        assert VioBridgeNode is not None


class TestVioEvaluatorImport:
    def test_import(self):
        from wardrone_vio.vio_evaluator_node import VioEvaluatorNode
        assert VioEvaluatorNode is not None
