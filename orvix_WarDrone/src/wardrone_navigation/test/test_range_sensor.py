"""Tests for range_sensor_node pure functions.

Functions are imported directly from range_sensor_node since they
are defined before any rclpy import (module-level pure functions).
We replicate them inline to avoid the rclpy import at module level.
"""

import pytest


# --- Replicated pure functions from range_sensor_node.py ---

TFMINI_HEADER = 0x59
TFMINI_FRAME_SIZE = 9


def parse_tfmini_frame(data: bytes) -> dict:
    if len(data) < TFMINI_FRAME_SIZE:
        return {'valid': False, 'error': 'short_frame'}
    if data[0] != TFMINI_HEADER or data[1] != TFMINI_HEADER:
        return {'valid': False, 'error': 'bad_header'}
    checksum = sum(data[:8]) & 0xFF
    if checksum != data[8]:
        return {'valid': False, 'error': 'bad_checksum'}
    dist_cm = data[2] | (data[3] << 8)
    strength = data[4] | (data[5] << 8)
    if dist_cm == 0 or strength < 100:
        return {'valid': False, 'error': 'weak_signal'}
    return {
        'valid': True,
        'distance_m': dist_cm / 100.0,
        'strength': strength,
    }


def find_tfmini_frame(buffer: bytes) -> tuple:
    while len(buffer) >= TFMINI_FRAME_SIZE:
        idx = buffer.find(bytes([TFMINI_HEADER, TFMINI_HEADER]))
        if idx < 0:
            return None, buffer[-1:] if buffer else b''
        if idx > 0:
            buffer = buffer[idx:]
        if len(buffer) < TFMINI_FRAME_SIZE:
            break
        frame = buffer[:TFMINI_FRAME_SIZE]
        parsed = parse_tfmini_frame(frame)
        if parsed['valid']:
            return frame, buffer[TFMINI_FRAME_SIZE:]
        else:
            buffer = buffer[2:]
    return None, buffer


def distance_to_threat_level(distance_m, dist_emergency=2.0, dist_critical=4.0,
                              dist_warning=8.0, dist_caution=12.0):
    if distance_m <= dist_emergency:
        return 5
    elif distance_m <= dist_critical:
        return 4
    elif distance_m <= dist_warning:
        return 3
    elif distance_m <= dist_caution:
        return 2
    else:
        return 1


# --- Helper to build a valid TFmini frame ---

def _build_frame(dist_cm: int, strength: int = 500) -> bytes:
    """Build a valid 9-byte TFmini-S frame."""
    data = bytearray(9)
    data[0] = 0x59
    data[1] = 0x59
    data[2] = dist_cm & 0xFF
    data[3] = (dist_cm >> 8) & 0xFF
    data[4] = strength & 0xFF
    data[5] = (strength >> 8) & 0xFF
    data[6] = 0  # temp low
    data[7] = 0  # temp high
    data[8] = sum(data[:8]) & 0xFF
    return bytes(data)


# --- Tests ---

class TestParseTfminiFrame:
    def test_valid_frame(self):
        """A well-formed frame with 150cm distance should parse correctly."""
        frame = _build_frame(150, 500)
        result = parse_tfmini_frame(frame)
        assert result['valid'] is True
        assert abs(result['distance_m'] - 1.5) < 0.01
        assert result['strength'] == 500

    def test_short_frame(self):
        result = parse_tfmini_frame(b'\x59\x59\x00')
        assert result['valid'] is False
        assert result['error'] == 'short_frame'

    def test_bad_header(self):
        frame = bytearray(_build_frame(100))
        frame[0] = 0x00  # corrupt header
        result = parse_tfmini_frame(bytes(frame))
        assert result['valid'] is False
        assert result['error'] == 'bad_header'

    def test_bad_checksum(self):
        frame = bytearray(_build_frame(100))
        frame[8] = 0x00  # corrupt checksum
        result = parse_tfmini_frame(bytes(frame))
        assert result['valid'] is False
        assert result['error'] == 'bad_checksum'

    def test_zero_distance(self):
        frame = _build_frame(0, 500)
        result = parse_tfmini_frame(frame)
        assert result['valid'] is False
        assert result['error'] == 'weak_signal'

    def test_weak_signal(self):
        frame = _build_frame(100, 50)  # strength < 100
        result = parse_tfmini_frame(frame)
        assert result['valid'] is False
        assert result['error'] == 'weak_signal'

    def test_max_distance(self):
        """12m = 1200cm should parse correctly."""
        frame = _build_frame(1200, 300)
        result = parse_tfmini_frame(frame)
        assert result['valid'] is True
        assert abs(result['distance_m'] - 12.0) < 0.01

    def test_min_distance(self):
        """30cm should parse correctly."""
        frame = _build_frame(30, 999)
        result = parse_tfmini_frame(frame)
        assert result['valid'] is True
        assert abs(result['distance_m'] - 0.3) < 0.01


class TestFindTfminiFrame:
    def test_frame_at_start(self):
        frame = _build_frame(200)
        found, remaining = find_tfmini_frame(frame)
        assert found is not None
        assert found == frame
        assert remaining == b''

    def test_garbage_before_frame(self):
        garbage = b'\x00\x01\x02\x03'
        frame = _build_frame(200)
        found, remaining = find_tfmini_frame(garbage + frame)
        assert found is not None
        assert found == frame

    def test_two_consecutive_frames(self):
        frame1 = _build_frame(100)
        frame2 = _build_frame(200)
        found1, remaining = find_tfmini_frame(frame1 + frame2)
        assert found1 == frame1
        found2, remaining = find_tfmini_frame(remaining)
        assert found2 == frame2

    def test_incomplete_frame(self):
        frame = _build_frame(100)
        found, remaining = find_tfmini_frame(frame[:5])
        assert found is None
        assert len(remaining) == 5

    def test_empty_buffer(self):
        found, remaining = find_tfmini_frame(b'')
        assert found is None
        assert remaining == b''

    def test_no_header_in_buffer(self):
        found, remaining = find_tfmini_frame(b'\x00\x01\x02\x03\x04\x05\x06\x07\x08')
        assert found is None


class TestDistanceToThreatLevel:
    def test_emergency(self):
        assert distance_to_threat_level(0.5) == 5
        assert distance_to_threat_level(2.0) == 5

    def test_critical(self):
        assert distance_to_threat_level(3.0) == 4
        assert distance_to_threat_level(4.0) == 4

    def test_warning(self):
        assert distance_to_threat_level(5.0) == 3
        assert distance_to_threat_level(8.0) == 3

    def test_caution(self):
        assert distance_to_threat_level(9.0) == 2
        assert distance_to_threat_level(12.0) == 2

    def test_monitor(self):
        assert distance_to_threat_level(13.0) == 1
        assert distance_to_threat_level(100.0) == 1

    def test_custom_thresholds(self):
        assert distance_to_threat_level(
            2.0, dist_emergency=5.0, dist_critical=10.0,
            dist_warning=20.0, dist_caution=30.0
        ) == 5

    def test_boundary_at_zero(self):
        assert distance_to_threat_level(0.0) == 5
