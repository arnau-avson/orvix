"""Simple HMAC authentication for critical ROS 2 messages.

Lightweight alternative to full SROS2 for academic/demonstration purposes.
For production use, enable ROS 2 DDS Security (SROS2) instead.

Usage:
    signed = sign_message(b"arm_command", key=b"shared_secret")
    valid, data = verify_message(signed, key=b"shared_secret")
"""

import hmac
import hashlib
import struct
import time


def sign_message(data: bytes, key: bytes) -> bytes:
    """Create HMAC-SHA256 signature for message data.

    Returns: timestamp (8B) + signature (32B) + original data
    """
    timestamp = struct.pack('>d', time.time())
    payload = timestamp + data
    signature = hmac.new(key, payload, hashlib.sha256).digest()
    return timestamp + signature + data


def verify_message(signed_data: bytes, key: bytes, max_age_s: float = 5.0) -> tuple:
    """Verify HMAC signature and message freshness.

    Returns: (valid: bool, original_data: bytes)
    """
    if len(signed_data) < 40:  # 8 (timestamp) + 32 (HMAC)
        return False, b''

    timestamp = struct.unpack('>d', signed_data[:8])[0]
    signature = signed_data[8:40]
    data = signed_data[40:]

    # Check freshness
    if abs(time.time() - timestamp) > max_age_s:
        return False, b''

    # Verify HMAC
    expected = hmac.new(key, signed_data[:8] + data, hashlib.sha256).digest()
    if hmac.compare_digest(signature, expected):
        return True, data
    return False, b''
