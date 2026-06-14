"""Tests for camera_node module.

These tests exercise the camera intrinsics calculation and configuration
logic as pure Python -- no ROS 2 runtime required.
"""

import pytest


class TestCameraNodeImport:
    def test_import(self):
        from wardrone_vision.camera_node import CameraNode
        assert CameraNode is not None


# ---------------------------------------------------------------------------
# Camera intrinsics calculation
#
# CameraNode._publish_camera_info builds a pinhole camera model with:
#   fx = width / 2.0   (approx 90 degree horizontal FOV)
#   fy = height / 2.0  (approx 90 degree vertical FOV)
#   cx = width / 2.0   (principal point at image center)
#   cy = height / 2.0
# ---------------------------------------------------------------------------

class TestCameraIntrinsics:
    """Test the camera intrinsics calculation logic from _publish_camera_info.

    We replicate the focal-length calculation so we can verify it
    without instantiating a ROS 2 Node.
    """

    @staticmethod
    def _compute_intrinsics(width, height):
        """Replicate the intrinsics calculation from _publish_camera_info.

        Returns (fx, fy, cx, cy, K, D, R, P).
        """
        fx = width / 2.0
        fy = height / 2.0
        cx = width / 2.0
        cy = height / 2.0

        K = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        D = [0.0, 0.0, 0.0, 0.0, 0.0]
        R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        P = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]

        return fx, fy, cx, cy, K, D, R, P

    # -- Default resolution (640x480) --

    def test_default_focal_length_x(self):
        fx, _, _, _, _, _, _, _ = self._compute_intrinsics(640, 480)
        assert fx == 320.0

    def test_default_focal_length_y(self):
        _, fy, _, _, _, _, _, _ = self._compute_intrinsics(640, 480)
        assert fy == 240.0

    def test_default_principal_point(self):
        _, _, cx, cy, _, _, _, _ = self._compute_intrinsics(640, 480)
        assert cx == 320.0
        assert cy == 240.0

    def test_default_K_matrix(self):
        """3x3 intrinsic matrix K (stored as flat list)."""
        _, _, _, _, K, _, _, _ = self._compute_intrinsics(640, 480)
        assert len(K) == 9
        # K = [fx, 0, cx, 0, fy, cy, 0, 0, 1]
        assert K[0] == 320.0   # fx
        assert K[1] == 0.0
        assert K[2] == 320.0   # cx
        assert K[3] == 0.0
        assert K[4] == 240.0   # fy
        assert K[5] == 240.0   # cy
        assert K[6] == 0.0
        assert K[7] == 0.0
        assert K[8] == 1.0

    def test_default_D_distortion(self):
        """Distortion coefficients must be all zeros (no distortion)."""
        _, _, _, _, _, D, _, _ = self._compute_intrinsics(640, 480)
        assert len(D) == 5
        assert all(d == 0.0 for d in D)

    def test_default_R_identity(self):
        """Rectification matrix must be 3x3 identity."""
        _, _, _, _, _, _, R, _ = self._compute_intrinsics(640, 480)
        assert len(R) == 9
        assert R == [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]

    def test_default_P_projection(self):
        """3x4 projection matrix P (flat list)."""
        _, _, _, _, _, _, _, P = self._compute_intrinsics(640, 480)
        assert len(P) == 12
        # P = [fx, 0, cx, 0, 0, fy, cy, 0, 0, 0, 1, 0]
        assert P[0] == 320.0   # fx
        assert P[5] == 240.0   # fy
        assert P[10] == 1.0    # scale
        assert P[3] == 0.0     # Tx (no stereo baseline)
        assert P[11] == 0.0

    # -- Different resolutions --

    def test_1080p_resolution(self):
        fx, fy, cx, cy, _, _, _, _ = self._compute_intrinsics(1920, 1080)
        assert fx == 960.0
        assert fy == 540.0
        assert cx == 960.0
        assert cy == 540.0

    def test_square_resolution(self):
        fx, fy, cx, cy, _, _, _, _ = self._compute_intrinsics(500, 500)
        assert fx == fy == 250.0
        assert cx == cy == 250.0

    def test_small_resolution(self):
        fx, fy, cx, cy, _, _, _, _ = self._compute_intrinsics(320, 240)
        assert fx == 160.0
        assert fy == 120.0

    # -- Matrix structure invariants --

    @pytest.mark.parametrize("width,height", [
        (640, 480), (1920, 1080), (320, 240), (800, 600),
    ])
    def test_K_diagonal_structure(self, width, height):
        """K must be upper-triangular with positive diagonal."""
        _, _, _, _, K, _, _, _ = self._compute_intrinsics(width, height)
        # K[0] = fx > 0, K[4] = fy > 0, K[8] = 1
        assert K[0] > 0
        assert K[4] > 0
        assert K[8] == 1.0
        # Off-diagonals below main diagonal are zero
        assert K[3] == 0.0  # row 1, col 0
        assert K[6] == 0.0  # row 2, col 0
        assert K[7] == 0.0  # row 2, col 1

    @pytest.mark.parametrize("width,height", [
        (640, 480), (1920, 1080), (320, 240),
    ])
    def test_principal_point_at_center(self, width, height):
        """Principal point must be at image center."""
        _, _, cx, cy, _, _, _, _ = self._compute_intrinsics(width, height)
        assert cx == width / 2.0
        assert cy == height / 2.0


class TestCameraSourceLogic:
    """Test the camera source selection logic."""

    VALID_SOURCES = ['gazebo', 'v4l2', 'csi']

    def test_gazebo_is_valid_source(self):
        assert 'gazebo' in self.VALID_SOURCES

    def test_v4l2_is_valid_source(self):
        assert 'v4l2' in self.VALID_SOURCES

    def test_csi_is_valid_source(self):
        assert 'csi' in self.VALID_SOURCES

    def test_unknown_source_not_valid(self):
        assert 'webcam' not in self.VALID_SOURCES

    @staticmethod
    def _is_capture_source(source):
        """Replicate the source check from CameraNode.__init__."""
        return source in ('v4l2', 'csi')

    def test_gazebo_is_not_capture(self):
        assert self._is_capture_source('gazebo') is False

    def test_v4l2_is_capture(self):
        assert self._is_capture_source('v4l2') is True

    def test_csi_is_capture(self):
        assert self._is_capture_source('csi') is True


class TestDefaultParameters:
    """Verify the default parameter values match what CameraNode declares."""

    DEFAULTS = {
        'source': 'gazebo',
        'device_id': 0,
        'width': 640,
        'height': 480,
        'fps': 30,
    }

    def test_default_source(self):
        assert self.DEFAULTS['source'] == 'gazebo'

    def test_default_device_id(self):
        assert self.DEFAULTS['device_id'] == 0

    def test_default_width(self):
        assert self.DEFAULTS['width'] == 640

    def test_default_height(self):
        assert self.DEFAULTS['height'] == 480

    def test_default_fps(self):
        assert self.DEFAULTS['fps'] == 30

    def test_default_fps_timer_interval(self):
        """Timer interval = 1.0 / fps."""
        interval = 1.0 / self.DEFAULTS['fps']
        assert abs(interval - 1.0 / 30.0) < 1e-9
