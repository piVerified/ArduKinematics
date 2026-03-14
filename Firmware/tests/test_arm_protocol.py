"""
test_arm_protocol.py – Unit tests for the shared arm_protocol module.

Run with:  python -m pytest Firmware/tests/
"""

import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "../shared"))
from arm_protocol import (
    FRAME_START, CMD_SET_JOINTS, CMD_STOP, CMD_HOME, CMD_GRIPPER,
    NUM_JOINTS,
    encode_set_joints, encode_stop, encode_home, encode_gripper,
    decode_frame, _checksum,
)


# ---------------------------------------------------------------------------
# Helper
# ---------------------------------------------------------------------------

def _valid_frame(data: bytes) -> bool:
    """Return True when *data* is a well-formed arm_protocol frame."""
    return decode_frame(data) is not None


# ---------------------------------------------------------------------------
# _checksum
# ---------------------------------------------------------------------------

class TestChecksum:
    def test_single_byte(self):
        assert _checksum(b'\xAB') == 0xAB

    def test_xor_property(self):
        data = bytes([0x01, 0x02, 0x03])
        assert _checksum(data) == (0x01 ^ 0x02 ^ 0x03)

    def test_self_inverse(self):
        data = b'\x01\x06' + bytes([90, 45, 135, 90, 90, 50])
        cs   = _checksum(data)
        # XOR-ing data with its own checksum yields 0 (self-inverse property)
        assert _checksum(data + bytes([cs])) == 0


# ---------------------------------------------------------------------------
# encode_stop / encode_home
# ---------------------------------------------------------------------------

class TestEncodeStopHome:
    def test_stop_starts_with_frame_start(self):
        frame = encode_stop()
        assert frame[0] == FRAME_START

    def test_stop_cmd_id(self):
        frame = encode_stop()
        assert frame[1] == CMD_STOP

    def test_stop_is_valid(self):
        assert _valid_frame(encode_stop())

    def test_home_starts_with_frame_start(self):
        frame = encode_home()
        assert frame[0] == FRAME_START

    def test_home_cmd_id(self):
        frame = encode_home()
        assert frame[1] == CMD_HOME

    def test_home_is_valid(self):
        assert _valid_frame(encode_home())


# ---------------------------------------------------------------------------
# encode_gripper
# ---------------------------------------------------------------------------

class TestEncodeGripper:
    def test_cmd_id(self):
        assert encode_gripper(0.5)[1] == CMD_GRIPPER

    def test_open_fully(self):
        frame = encode_gripper(1.0)
        assert _valid_frame(frame)
        _, payload = decode_frame(frame)
        assert payload[0] == 100

    def test_closed(self):
        frame = encode_gripper(0.0)
        _, payload = decode_frame(frame)
        assert payload[0] == 0

    def test_midpoint(self):
        frame = encode_gripper(0.5)
        _, payload = decode_frame(frame)
        assert payload[0] == 50

    def test_clamp_above_one(self):
        frame = encode_gripper(2.0)
        _, payload = decode_frame(frame)
        assert payload[0] == 100

    def test_clamp_below_zero(self):
        frame = encode_gripper(-1.0)
        _, payload = decode_frame(frame)
        assert payload[0] == 0


# ---------------------------------------------------------------------------
# encode_set_joints
# ---------------------------------------------------------------------------

class TestEncodeSetJoints:
    def _angles(self):
        return [90.0, 45.0, 135.0, 90.0, 90.0]

    def test_cmd_id(self):
        frame = encode_set_joints(self._angles())
        assert frame[1] == CMD_SET_JOINTS

    def test_is_valid(self):
        assert _valid_frame(encode_set_joints(self._angles()))

    def test_payload_length(self):
        frame = encode_set_joints(self._angles(), gripper_pct=0.5)
        _, payload = decode_frame(frame)
        # 5 joints + 1 gripper byte
        assert len(payload) == NUM_JOINTS + 1

    def test_joint_values_encoded(self):
        angles = [10.0, 20.0, 30.0, 40.0, 50.0]
        frame  = encode_set_joints(angles)
        _, payload = decode_frame(frame)
        assert list(payload[:NUM_JOINTS]) == [10, 20, 30, 40, 50]

    def test_gripper_encoded(self):
        frame = encode_set_joints(self._angles(), gripper_pct=0.75)
        _, payload = decode_frame(frame)
        assert payload[NUM_JOINTS] == 75

    def test_angle_clamp_high(self):
        frame = encode_set_joints([200.0] * NUM_JOINTS)
        _, payload = decode_frame(frame)
        assert all(b == 180 for b in payload[:NUM_JOINTS])

    def test_angle_clamp_low(self):
        frame = encode_set_joints([-50.0] * NUM_JOINTS)
        _, payload = decode_frame(frame)
        assert all(b == 0 for b in payload[:NUM_JOINTS])

    def test_wrong_joint_count_raises(self):
        import pytest
        with pytest.raises(ValueError):
            encode_set_joints([90.0] * 3)


# ---------------------------------------------------------------------------
# decode_frame – error cases
# ---------------------------------------------------------------------------

class TestDecodeFrameErrors:
    def test_too_short(self):
        assert decode_frame(b'\xAF') is None

    def test_wrong_start_byte(self):
        frame = encode_stop()
        bad   = bytes([0x00]) + frame[1:]
        assert decode_frame(bad) is None

    def test_corrupted_checksum(self):
        frame = bytearray(encode_stop())
        frame[-1] ^= 0xFF   # flip all bits in checksum
        assert decode_frame(bytes(frame)) is None

    def test_incomplete_payload(self):
        frame = encode_set_joints([90.0] * NUM_JOINTS)
        # Truncate before payload ends
        assert decode_frame(frame[:5]) is None


# ---------------------------------------------------------------------------
# Round-trip: encode → decode
# ---------------------------------------------------------------------------

class TestRoundTrip:
    def test_stop_round_trip(self):
        cmd, payload = decode_frame(encode_stop())
        assert cmd == CMD_STOP
        assert payload == b''

    def test_home_round_trip(self):
        cmd, payload = decode_frame(encode_home())
        assert cmd == CMD_HOME
        assert payload == b''

    def test_gripper_round_trip(self):
        cmd, payload = decode_frame(encode_gripper(0.3))
        assert cmd == CMD_GRIPPER
        assert payload[0] == 30

    def test_joints_round_trip(self):
        angles = [0.0, 45.0, 90.0, 135.0, 180.0]
        cmd, payload = decode_frame(encode_set_joints(angles, gripper_pct=1.0))
        assert cmd == CMD_SET_JOINTS
        assert list(payload[:NUM_JOINTS]) == [0, 45, 90, 135, 180]
        assert payload[NUM_JOINTS] == 100
