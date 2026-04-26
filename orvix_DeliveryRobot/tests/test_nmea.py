"""NMEA parser tests."""
import pytest

from delivery_robot.hardware.gps import parse_gpgga, parse_gprmc


class TestGPGGA:
    def test_valid_sentence(self):
        s = "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47"
        out = parse_gpgga(s)
        assert out is not None
        assert out["lat"] == pytest.approx(48.117297, abs=1e-4)
        assert out["lon"] == pytest.approx(11.516667, abs=1e-4)
        assert out["accuracy_m"] == pytest.approx(3.6, abs=0.1)  # 0.9 hdop * 4

    def test_no_fix(self):
        s = "$GPGGA,123519,,,,,0,00,99.9,,,,,,*48"
        assert parse_gpgga(s) is None

    def test_southern_hemisphere(self):
        s = "$GPGGA,000000,3415.000,S,07045.000,W,1,08,1.0,0,M,0,M,,*00"
        out = parse_gpgga(s)
        assert out["lat"] < 0
        assert out["lon"] < 0

    def test_wrong_sentence_type(self):
        assert parse_gpgga("$GPRMC,...") is None

    def test_malformed(self):
        assert parse_gpgga("not a sentence") is None
        assert parse_gpgga("$GPGGA,1,2,3") is None  # too few fields

    def test_gnss_variant(self):
        # $GNGGA is the multi-constellation flavor
        s = "$GNGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47"
        assert parse_gpgga(s) is not None


class TestGPRMC:
    def test_valid_active(self):
        s = "$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A"
        out = parse_gprmc(s)
        assert out is not None
        assert out["lat"] == pytest.approx(48.117297, abs=1e-4)
        assert out["heading_deg"] == pytest.approx(84.4, abs=0.1)
        # 22.4 knots * 0.5144 ≈ 11.52 m/s
        assert out["speed_mps"] == pytest.approx(11.52, abs=0.1)

    def test_void_status(self):
        # 'V' = warning / no fix
        s = "$GPRMC,000000,V,,,,,,,,,*42"
        assert parse_gprmc(s) is None

    def test_empty_heading_returns_none(self):
        s = "$GPRMC,000000,A,4807.038,N,01131.000,E,000.0,,230394,,*00"
        out = parse_gprmc(s)
        assert out is not None
        assert out["heading_deg"] is None
