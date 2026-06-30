"""Tests for obstacle_merger_node pure functions.

The merge_obstacle_arrays function is replicated inline to avoid
importing obstacle_merger_node.py which requires rclpy at module level.
"""

import pytest


# --- Replicated pure functions from obstacle_merger_node.py ---

def _to_dict(obs) -> dict:
    if isinstance(obs, dict):
        return dict(obs)
    return {}


def merge_obstacle_arrays(vision_obstacles, range_obstacles,
                           vision_age_s, range_age_s, max_age_s=1.0):
    result = []
    v_obs = vision_obstacles if vision_age_s <= max_age_s else []
    r_obs = range_obstacles if range_age_s <= max_age_s else []

    range_by_sector = {}
    for obs in r_obs:
        sector = obs.get('sector', '') if isinstance(obs, dict) else obs.sector
        range_by_sector[sector] = obs

    fused_sectors = set()
    for v in v_obs:
        sector = v.get('sector', '') if isinstance(v, dict) else v.sector
        r = range_by_sector.get(sector)
        if r is not None:
            fused = dict(v)
            r_dict = dict(r)
            fused['estimated_distance_m'] = r_dict['estimated_distance_m']
            fused['approach_velocity_m_s'] = r_dict['approach_velocity_m_s']
            fused['time_to_collision_s'] = r_dict['time_to_collision_s']
            fused['threat_level'] = max(r_dict['threat_level'], fused['threat_level'])
            fused['source'] = 'fused'
            result.append(fused)
            fused_sectors.add(sector)
        else:
            d = dict(v)
            d['source'] = 'vision'
            result.append(d)

    for sector, r in range_by_sector.items():
        if sector not in fused_sectors:
            d = dict(r)
            d['source'] = 'range'
            result.append(d)

    return result


# --- Helpers ---

def _make_obstacle(sector='FRONT', distance=5.0, classification='tree',
                    threat_level=3, approach_vel=0.0, ttc=-1.0):
    return {
        'sector': sector,
        'bearing_deg': 0.0,
        'estimated_distance_m': distance,
        'approach_velocity_m_s': approach_vel,
        'time_to_collision_s': ttc,
        'classification': classification,
        'classification_confidence': 0.8,
        'threat_level': threat_level,
    }


# --- Tests ---

class TestMergeObstacleArrays:
    def test_vision_only(self):
        """With only vision data, all obstacles pass through as source=vision."""
        v = [_make_obstacle('FRONT', 8.0, 'tree', 3)]
        result = merge_obstacle_arrays(v, [], 0.1, 999.0)
        assert len(result) == 1
        assert result[0]['source'] == 'vision'
        assert result[0]['classification'] == 'tree'
        assert result[0]['estimated_distance_m'] == 8.0

    def test_range_only(self):
        """With only range data, obstacles pass through as source=range."""
        r = [_make_obstacle('FRONT', 3.0, 'unknown', 4)]
        result = merge_obstacle_arrays([], r, 999.0, 0.1)
        assert len(result) == 1
        assert result[0]['source'] == 'range'
        assert result[0]['estimated_distance_m'] == 3.0

    def test_same_sector_fuses(self):
        """When both sources have FRONT, range distance replaces vision estimate,
        but vision classification is kept."""
        v = [_make_obstacle('FRONT', 10.0, 'building', 2)]
        r = [_make_obstacle('FRONT', 5.0, 'unknown', 4)]
        result = merge_obstacle_arrays(v, r, 0.1, 0.1)
        assert len(result) == 1
        assert result[0]['source'] == 'fused'
        assert result[0]['estimated_distance_m'] == 5.0  # Range distance
        assert result[0]['classification'] == 'building'  # Vision class
        assert result[0]['threat_level'] == 4  # max(2, 4)

    def test_different_sectors_no_fusion(self):
        """Vision=FRONT, Range=LEFT -> two separate obstacles, no fusion."""
        v = [_make_obstacle('FRONT', 8.0, 'tree', 3)]
        r = [_make_obstacle('LEFT', 3.0, 'unknown', 4)]
        result = merge_obstacle_arrays(v, r, 0.1, 0.1)
        assert len(result) == 2
        sectors = {o['sector'] for o in result}
        assert sectors == {'FRONT', 'LEFT'}
        front = [o for o in result if o['sector'] == 'FRONT'][0]
        left = [o for o in result if o['sector'] == 'LEFT'][0]
        assert front['source'] == 'vision'
        assert left['source'] == 'range'

    def test_stale_vision_discarded(self):
        """Vision older than max_age is ignored."""
        v = [_make_obstacle('FRONT', 8.0)]
        r = [_make_obstacle('FRONT', 3.0)]
        result = merge_obstacle_arrays(v, r, 2.0, 0.1, max_age_s=1.0)
        assert len(result) == 1
        assert result[0]['source'] == 'range'

    def test_stale_range_discarded(self):
        """Range older than max_age is ignored."""
        v = [_make_obstacle('FRONT', 8.0)]
        r = [_make_obstacle('FRONT', 3.0)]
        result = merge_obstacle_arrays(v, r, 0.1, 2.0, max_age_s=1.0)
        assert len(result) == 1
        assert result[0]['source'] == 'vision'

    def test_both_stale_empty(self):
        """If both sources are stale, result is empty."""
        v = [_make_obstacle('FRONT', 8.0)]
        r = [_make_obstacle('FRONT', 3.0)]
        result = merge_obstacle_arrays(v, r, 2.0, 2.0, max_age_s=1.0)
        assert len(result) == 0

    def test_empty_inputs(self):
        """No obstacles from either source -> empty result."""
        result = merge_obstacle_arrays([], [], 0.1, 0.1)
        assert len(result) == 0

    def test_threat_level_takes_max(self):
        """Fused threat level should be max of both sources."""
        v = [_make_obstacle('FRONT', 10.0, 'tree', 5)]  # vision says 5
        r = [_make_obstacle('FRONT', 5.0, 'unknown', 3)]  # range says 3
        result = merge_obstacle_arrays(v, r, 0.1, 0.1)
        assert result[0]['threat_level'] == 5  # max(5, 3)

    def test_multiple_vision_one_range(self):
        """Multiple vision obstacles, one matching range sector."""
        v = [
            _make_obstacle('FRONT', 10.0, 'bird', 2),
            _make_obstacle('RIGHT', 15.0, 'tree', 1),
        ]
        r = [_make_obstacle('FRONT', 4.0, 'unknown', 4)]
        result = merge_obstacle_arrays(v, r, 0.1, 0.1)
        assert len(result) == 2
        front = [o for o in result if o['sector'] == 'FRONT'][0]
        right = [o for o in result if o['sector'] == 'RIGHT'][0]
        assert front['source'] == 'fused'
        assert front['estimated_distance_m'] == 4.0
        assert front['classification'] == 'bird'
        assert right['source'] == 'vision'

    def test_approach_velocity_from_range(self):
        """Fused obstacle uses range sensor's approach velocity."""
        v = [_make_obstacle('FRONT', 10.0, 'tree', 2, approach_vel=0.0)]
        r = [_make_obstacle('FRONT', 5.0, 'unknown', 3, approach_vel=2.5, ttc=2.0)]
        result = merge_obstacle_arrays(v, r, 0.1, 0.1)
        assert abs(result[0]['approach_velocity_m_s'] - 2.5) < 1e-9
        assert abs(result[0]['time_to_collision_s'] - 2.0) < 1e-9


class TestMultiSensorMerge:
    """Test merger with multiple range sensors covering different sectors."""

    def test_three_range_sensors(self):
        """Three range sensors (FRONT, LEFT, RIGHT) merge independently."""
        v = [_make_obstacle('FRONT', 10.0, 'bird', 2)]
        r = [
            _make_obstacle('FRONT', 4.0, 'unknown', 4),
            _make_obstacle('LEFT', 6.0, 'unknown', 3),
            _make_obstacle('RIGHT', 8.0, 'unknown', 2),
        ]
        result = merge_obstacle_arrays(v, r, 0.1, 0.1)
        assert len(result) == 3
        front = [o for o in result if o['sector'] == 'FRONT'][0]
        left = [o for o in result if o['sector'] == 'LEFT'][0]
        right = [o for o in result if o['sector'] == 'RIGHT'][0]
        assert front['source'] == 'fused'
        assert front['estimated_distance_m'] == 4.0
        assert front['classification'] == 'bird'
        assert left['source'] == 'range'
        assert right['source'] == 'range'

    def test_multiple_range_same_sector_last_wins(self):
        """If two range obstacles share a sector, dict indexing keeps the last."""
        r = [
            _make_obstacle('FRONT', 5.0, 'unknown', 3),
            _make_obstacle('FRONT', 3.0, 'unknown', 4),
        ]
        result = merge_obstacle_arrays([], r, 999.0, 0.1)
        assert len(result) == 1
        assert result[0]['estimated_distance_m'] == 3.0

    def test_vision_two_sectors_range_two_sectors(self):
        """Vision on FRONT+RIGHT, range on FRONT+LEFT."""
        v = [
            _make_obstacle('FRONT', 15.0, 'drone', 2),
            _make_obstacle('RIGHT', 20.0, 'tree', 1),
        ]
        r = [
            _make_obstacle('FRONT', 5.0, 'unknown', 4),
            _make_obstacle('LEFT', 7.0, 'unknown', 3),
        ]
        result = merge_obstacle_arrays(v, r, 0.1, 0.1)
        assert len(result) == 3
        front = [o for o in result if o['sector'] == 'FRONT'][0]
        right = [o for o in result if o['sector'] == 'RIGHT'][0]
        left = [o for o in result if o['sector'] == 'LEFT'][0]
        assert front['source'] == 'fused'
        assert front['estimated_distance_m'] == 5.0
        assert front['classification'] == 'drone'
        assert right['source'] == 'vision'
        assert left['source'] == 'range'

    def test_all_sensors_all_sectors(self):
        """Vision + 3 range sensors covering 4 different sectors total."""
        v = [
            _make_obstacle('FRONT', 12.0, 'building', 2),
            _make_obstacle('REAR', 25.0, 'tree', 1),
        ]
        r = [
            _make_obstacle('FRONT', 6.0, 'unknown', 3),
            _make_obstacle('LEFT', 8.0, 'unknown', 2),
            _make_obstacle('RIGHT', 10.0, 'unknown', 2),
        ]
        result = merge_obstacle_arrays(v, r, 0.1, 0.1)
        assert len(result) == 4  # FRONT(fused), REAR(vision), LEFT(range), RIGHT(range)
        sources = {o['sector']: o['source'] for o in result}
        assert sources['FRONT'] == 'fused'
        assert sources['REAR'] == 'vision'
        assert sources['LEFT'] == 'range'
        assert sources['RIGHT'] == 'range'


class TestPerSectorBuffering:
    """Test the per-sector range buffering logic used in _on_range."""

    @staticmethod
    def _buffer(range_by_sector, time_by_sector, obstacles, now_ns):
        """Replicate the per-sector buffering from obstacle_merger_node."""
        for obs in obstacles:
            sector = obs['sector']
            range_by_sector[sector] = obs
            time_by_sector[sector] = now_ns

    def test_single_sensor(self):
        by_sector = {}
        time_by_sector = {}
        obs = [_make_obstacle('FRONT', 5.0)]
        self._buffer(by_sector, time_by_sector, obs, 1000)
        assert 'FRONT' in by_sector
        assert by_sector['FRONT']['estimated_distance_m'] == 5.0

    def test_multiple_sensors_buffered_separately(self):
        by_sector = {}
        time_by_sector = {}
        # Sensor 1 publishes FRONT
        self._buffer(by_sector, time_by_sector,
                     [_make_obstacle('FRONT', 5.0)], 1000)
        # Sensor 2 publishes LEFT
        self._buffer(by_sector, time_by_sector,
                     [_make_obstacle('LEFT', 7.0)], 1001)
        assert len(by_sector) == 2
        assert 'FRONT' in by_sector
        assert 'LEFT' in by_sector
        assert by_sector['FRONT']['estimated_distance_m'] == 5.0
        assert by_sector['LEFT']['estimated_distance_m'] == 7.0

    def test_same_sector_update_replaces(self):
        by_sector = {}
        time_by_sector = {}
        self._buffer(by_sector, time_by_sector,
                     [_make_obstacle('FRONT', 5.0)], 1000)
        self._buffer(by_sector, time_by_sector,
                     [_make_obstacle('FRONT', 3.0)], 2000)
        assert by_sector['FRONT']['estimated_distance_m'] == 3.0
        assert time_by_sector['FRONT'] == 2000

    def test_three_sensors_all_buffered(self):
        by_sector = {}
        time_by_sector = {}
        self._buffer(by_sector, time_by_sector,
                     [_make_obstacle('FRONT', 4.0)], 100)
        self._buffer(by_sector, time_by_sector,
                     [_make_obstacle('LEFT', 6.0)], 101)
        self._buffer(by_sector, time_by_sector,
                     [_make_obstacle('RIGHT', 8.0)], 102)
        assert len(by_sector) == 3
        assert set(by_sector.keys()) == {'FRONT', 'LEFT', 'RIGHT'}
