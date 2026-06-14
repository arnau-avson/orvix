"""Tests for mission_loader module."""

import os
import tempfile
import pytest
import yaml

from wardrone_navigation.mission_loader import (
    load_mission, validate_mission, MissionLoadError, MissionData, WaypointData
)


def _write_yaml(data: dict) -> str:
    """Helper: write dict to a temp YAML file and return path."""
    fd, path = tempfile.mkstemp(suffix='.yaml')
    with os.fdopen(fd, 'w') as f:
        yaml.dump(data, f)
    return path


class TestLoadMission:
    def test_valid_mission(self):
        data = {
            'mission': {
                'id': 'test_01',
                'default_altitude_m': 15.0,
                'default_speed_m_s': 3.0,
                'waypoints': [
                    {'latitude_deg': 47.3977, 'longitude_deg': 8.5456, 'altitude_m': 10.0},
                    {'latitude_deg': 47.3987, 'longitude_deg': 8.5466},
                ],
            }
        }
        path = _write_yaml(data)
        try:
            mission = load_mission(path)
            assert mission.mission_id == 'test_01'
            assert len(mission.waypoints) == 2
            assert mission.waypoints[0].altitude_m == 10.0
            assert mission.waypoints[1].altitude_m == 15.0  # Uses default
            assert mission.default_speed_m_s == 3.0
        finally:
            os.unlink(path)

    def test_missing_file(self):
        with pytest.raises(MissionLoadError, match="not found"):
            load_mission("/nonexistent/path.yaml")

    def test_missing_mission_key(self):
        path = _write_yaml({'waypoints': []})
        try:
            with pytest.raises(MissionLoadError, match="Missing top-level"):
                load_mission(path)
        finally:
            os.unlink(path)

    def test_no_waypoints(self):
        path = _write_yaml({'mission': {'id': 'empty', 'waypoints': []}})
        try:
            with pytest.raises(MissionLoadError, match="no waypoints"):
                load_mission(path)
        finally:
            os.unlink(path)

    def test_missing_lat_lon(self):
        data = {'mission': {'waypoints': [{'altitude_m': 10.0}]}}
        path = _write_yaml(data)
        try:
            with pytest.raises(MissionLoadError, match="missing latitude"):
                load_mission(path)
        finally:
            os.unlink(path)

    def test_latitude_out_of_range(self):
        data = {'mission': {'waypoints': [{'latitude_deg': 95.0, 'longitude_deg': 0.0}]}}
        path = _write_yaml(data)
        try:
            with pytest.raises(MissionLoadError, match="out of range"):
                load_mission(path)
        finally:
            os.unlink(path)

    def test_default_mission_id(self):
        data = {'mission': {'waypoints': [{'latitude_deg': 0.0, 'longitude_deg': 0.0}]}}
        path = _write_yaml(data)
        try:
            mission = load_mission(path)
            assert mission.mission_id == 'unnamed'
        finally:
            os.unlink(path)


class TestValidateMission:
    def test_valid(self):
        mission = MissionData(
            mission_id='ok',
            waypoints=[WaypointData(47.0, 8.0, 10.0)],
        )
        warnings = validate_mission(mission)
        assert len(warnings) == 0

    def test_low_altitude_warning(self):
        mission = MissionData(
            mission_id='low',
            waypoints=[WaypointData(47.0, 8.0, 0.5)],
        )
        warnings = validate_mission(mission)
        assert any('very low' in w for w in warnings)

    def test_high_altitude_warning(self):
        mission = MissionData(
            mission_id='high',
            waypoints=[WaypointData(47.0, 8.0, 150.0)],
        )
        warnings = validate_mission(mission)
        assert any('120m' in w for w in warnings)
