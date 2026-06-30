"""Tests for auth_utils HMAC message signing and verification."""

import time
import pytest

from wardrone_navigation.auth_utils import sign_message, verify_message


class TestSignMessage:
    def test_output_length(self):
        """Signed output should be 8 (timestamp) + 32 (HMAC-SHA256) + len(data)."""
        data = b"hello world"
        key = b"secret_key"
        signed = sign_message(data, key)
        assert len(signed) == 8 + 32 + len(data)

    def test_original_data_preserved(self):
        """The original data should be preserved at the end of the signed message."""
        data = b"arm_command"
        key = b"my_key"
        signed = sign_message(data, key)
        assert signed[40:] == data

    def test_different_data_different_signature(self):
        """Different data should produce different signatures."""
        key = b"secret"
        signed1 = sign_message(b"command_a", key)
        signed2 = sign_message(b"command_b", key)
        # Signatures are bytes 8:40
        assert signed1[8:40] != signed2[8:40]

    def test_different_keys_different_signature(self):
        """Different keys should produce different signatures."""
        data = b"same_data"
        signed1 = sign_message(data, b"key_one")
        signed2 = sign_message(data, b"key_two")
        assert signed1[8:40] != signed2[8:40]

    def test_empty_data(self):
        """Signing empty data should work."""
        signed = sign_message(b"", b"key")
        assert len(signed) == 8 + 32 + 0


class TestVerifyMessage:
    def test_valid_signature(self):
        """A freshly signed message should verify correctly."""
        data = b"test_payload"
        key = b"shared_secret"
        signed = sign_message(data, key)
        valid, recovered_data = verify_message(signed, key)
        assert valid is True
        assert recovered_data == data

    def test_wrong_key(self):
        """Verification with a wrong key should fail."""
        data = b"important_command"
        signed = sign_message(data, b"correct_key")
        valid, recovered_data = verify_message(signed, b"wrong_key")
        assert valid is False
        assert recovered_data == b''

    def test_tampered_data(self):
        """Modifying the data portion should fail verification."""
        data = b"original_data"
        key = b"key"
        signed = sign_message(data, key)
        # Tamper with the last byte of the data
        tampered = signed[:-1] + bytes([(signed[-1] + 1) % 256])
        valid, recovered_data = verify_message(tampered, key)
        assert valid is False
        assert recovered_data == b''

    def test_truncated_data(self):
        """A truncated message (less than 40 bytes) should fail."""
        valid, recovered_data = verify_message(b"short", b"key")
        assert valid is False
        assert recovered_data == b''

    def test_truncated_at_boundary(self):
        """Exactly 39 bytes (below minimum 40) should fail."""
        valid, recovered_data = verify_message(b"x" * 39, b"key")
        assert valid is False
        assert recovered_data == b''

    def test_empty_data_round_trip(self):
        """Signing and verifying empty data should work."""
        key = b"key"
        signed = sign_message(b"", key)
        valid, recovered_data = verify_message(signed, key)
        assert valid is True
        assert recovered_data == b""


class TestMessageFreshness:
    def test_expired_message(self):
        """A message older than max_age should fail verification."""
        data = b"time_sensitive"
        key = b"secret"
        signed = sign_message(data, key)
        # Wait for the message to expire
        time.sleep(0.15)
        valid, recovered_data = verify_message(signed, key, max_age_s=0.05)
        assert valid is False
        assert recovered_data == b''

    def test_fresh_message(self):
        """A fresh message within max_age should pass."""
        data = b"still_valid"
        key = b"secret"
        signed = sign_message(data, key)
        valid, recovered_data = verify_message(signed, key, max_age_s=10.0)
        assert valid is True
        assert recovered_data == data
