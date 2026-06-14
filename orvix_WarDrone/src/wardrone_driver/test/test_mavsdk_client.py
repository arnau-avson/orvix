import math
import pytest
from wardrone_driver.mavsdk_client import MavsdkClient, TelemetryData, FlightMode


class TestCoordinateTransforms:
    def test_enu_to_ned(self):
        north, east, down = MavsdkClient.enu_to_ned(1.0, 2.0, 3.0)
        assert north == pytest.approx(2.0)
        assert east == pytest.approx(1.0)
        assert down == pytest.approx(-3.0)

    def test_ned_to_enu(self):
        east, north, up = MavsdkClient.ned_to_enu(2.0, 1.0, -3.0)
        assert east == pytest.approx(1.0)
        assert north == pytest.approx(2.0)
        assert up == pytest.approx(3.0)

    def test_enu_ned_roundtrip(self):
        x, y, z = 5.0, 10.0, 15.0
        n, e, d = MavsdkClient.enu_to_ned(x, y, z)
        x2, y2, z2 = MavsdkClient.ned_to_enu(n, e, d)
        assert x2 == pytest.approx(x)
        assert y2 == pytest.approx(y)
        assert z2 == pytest.approx(z)

    def test_yaw_enu_to_ned(self):
        # North in ENU is 90 deg, in NED is 0 deg
        yaw_ned = MavsdkClient.yaw_enu_to_ned(math.pi / 2.0)
        assert yaw_ned == pytest.approx(0.0)

    def test_yaw_ned_to_enu(self):
        yaw_enu = MavsdkClient.yaw_ned_to_enu(0.0)
        assert yaw_enu == pytest.approx(math.pi / 2.0)

    def test_yaw_roundtrip(self):
        yaw_orig = 1.23
        yaw_ned = MavsdkClient.yaw_enu_to_ned(yaw_orig)
        yaw_back = MavsdkClient.yaw_ned_to_enu(yaw_ned)
        assert yaw_back == pytest.approx(yaw_orig)


class TestTelemetryData:
    def test_default_values(self):
        t = TelemetryData()
        assert t.latitude_deg == 0.0
        assert t.is_armed is False
        assert t.flight_mode == FlightMode.UNKNOWN

    def test_flight_modes(self):
        assert FlightMode.HOLD.value == "HOLD"
        assert FlightMode.OFFBOARD.value == "OFFBOARD"
        assert FlightMode.RTL.value == "RTL"


class TestMavsdkClient:
    def test_initial_state(self):
        client = MavsdkClient()
        assert client.connected is False
        assert client.telemetry.is_armed is False

    def test_custom_connection_url(self):
        client = MavsdkClient(connection_url="serial:///dev/ttyACM0:921600")
        assert client._connection_url == "serial:///dev/ttyACM0:921600"

    def test_telemetry_callback(self):
        client = MavsdkClient()
        received = []
        client.add_telemetry_callback(lambda t: received.append(t))
        client._telemetry.is_armed = True
        client._notify_telemetry()
        assert len(received) == 1
        assert received[0].is_armed is True
