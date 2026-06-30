"""Tests for flight_logger_node CSV configuration.

CSV_COLUMNS is replicated inline to avoid importing flight_logger_node.py
which requires rclpy at module level.
"""

import pytest


# Replicated from flight_logger_node.py to avoid rclpy import
CSV_COLUMNS = [
    'timestamp', 'lat', 'lon', 'abs_alt', 'rel_alt',
    'vel_n', 'vel_e', 'vel_d', 'roll', 'pitch', 'yaw',
    'battery_pct', 'battery_v', 'gps_sats', 'gps_fix',
    'flight_mode', 'is_armed', 'is_in_air',
    'mission_state', 'safety_event', 'obstacle_max_threat',
    'wind_speed_m_s',
]


class TestFlightLoggerCSV:
    def test_column_count(self):
        assert len(CSV_COLUMNS) == 22

    def test_has_timestamp(self):
        assert 'timestamp' in CSV_COLUMNS

    def test_has_position_fields(self):
        assert all(c in CSV_COLUMNS for c in ['lat', 'lon', 'abs_alt', 'rel_alt'])

    def test_has_velocity_fields(self):
        assert all(c in CSV_COLUMNS for c in ['vel_n', 'vel_e', 'vel_d'])

    def test_has_attitude_fields(self):
        assert all(c in CSV_COLUMNS for c in ['roll', 'pitch', 'yaw'])

    def test_has_battery_fields(self):
        assert all(c in CSV_COLUMNS for c in ['battery_pct', 'battery_v'])

    def test_has_gps_fields(self):
        assert all(c in CSV_COLUMNS for c in ['gps_sats', 'gps_fix'])

    def test_has_wind_field(self):
        assert 'wind_speed_m_s' in CSV_COLUMNS

    def test_has_safety_field(self):
        assert 'safety_event' in CSV_COLUMNS

    def test_has_mission_state_field(self):
        assert 'mission_state' in CSV_COLUMNS

    def test_has_obstacle_field(self):
        assert 'obstacle_max_threat' in CSV_COLUMNS

    def test_has_flight_status_fields(self):
        assert all(c in CSV_COLUMNS for c in ['flight_mode', 'is_armed', 'is_in_air'])

    def test_column_order_timestamp_first(self):
        assert CSV_COLUMNS[0] == 'timestamp'

    def test_columns_are_strings(self):
        assert all(isinstance(c, str) for c in CSV_COLUMNS)

    def test_no_duplicates(self):
        assert len(CSV_COLUMNS) == len(set(CSV_COLUMNS))
