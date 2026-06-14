"""Tests for mission_controller_node module."""

import pytest


class TestMissionControllerImport:
    def test_import(self):
        from wardrone_mission.mission_controller_node import MissionControllerNode
        assert MissionControllerNode is not None

    def test_import_states(self):
        from wardrone_mission.states import MissionState, MissionEvent, MissionType
        assert MissionState.IDLE.value == "IDLE"
        assert MissionEvent.CMD_START.value == "CMD_START"
        assert MissionType.NAVIGATE_AND_TRACK.value == "navigate_and_track"
