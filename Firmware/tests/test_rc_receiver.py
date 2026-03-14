"""
test_rc_receiver.py – Unit tests for the RC receiver parser.
"""

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "../hover"))

from rc_receiver import RCReceiver, _normalise, _normalise_pos, _raw_to_angle, _CH_MIN, _CH_MID, _CH_MAX


class TestNormalise:
    def test_mid(self):
        assert _normalise(_CH_MID) == 0.0

    def test_min(self):
        assert _normalise(_CH_MIN) == -1.0

    def test_max(self):
        assert _normalise(_CH_MAX) == 1.0

    def test_clamp_high(self):
        assert _normalise(3000) == 1.0

    def test_clamp_low(self):
        assert _normalise(0) == -1.0


class TestNormalisePos:
    def test_min(self):
        assert _normalise_pos(_CH_MIN) == 0.0

    def test_max(self):
        assert _normalise_pos(_CH_MAX) == 1.0

    def test_mid(self):
        assert abs(_normalise_pos(_CH_MID) - 0.5) < 1e-9


class TestRawToAngle:
    def test_min_gives_zero(self):
        assert _raw_to_angle(_CH_MIN) == 0.0

    def test_max_gives_180(self):
        assert _raw_to_angle(_CH_MAX) == 180.0

    def test_mid_gives_90(self):
        assert abs(_raw_to_angle(_CH_MID) - 90.0) < 1e-6

    def test_custom_max(self):
        assert _raw_to_angle(_CH_MAX, max_deg=90.0) == 90.0


class TestRCReceiver:
    def _make_receiver(self, channel_values=None):
        """Create an RCReceiver with a mock UART and preset channels."""
        class MockUART:
            def any(self):
                return 0
            def read(self, n):
                return b''

        rc = RCReceiver(MockUART())
        if channel_values:
            rc._channels = channel_values
        return rc

    def test_get_channels_default(self):
        rc = self._make_receiver()
        throttle, roll, pitch, yaw = rc.get_channels()
        assert throttle == 0.5    # CH3 defaults to _CH_MID → 0.5
        assert roll  == 0.0
        assert pitch == 0.0
        assert yaw   == 0.0

    def test_get_arm_channels_returns_five(self):
        rc = self._make_receiver()
        joints = rc.get_arm_channels()
        assert len(joints) == 5

    def test_get_arm_channels_mid_gives_90(self):
        rc = self._make_receiver()
        joints = rc.get_arm_channels()
        for angle in joints:
            assert abs(angle - 90.0) < 1e-6

    def test_get_gripper_default(self):
        rc = self._make_receiver()
        assert abs(rc.get_gripper() - 0.5) < 1e-9

    def test_arm_channels_min_gives_zero(self):
        vals = [_CH_MID] * 10
        for i in range(4, 9):
            vals[i] = _CH_MIN
        rc = self._make_receiver(vals)
        for angle in rc.get_arm_channels():
            assert angle == 0.0

    def test_arm_channels_max_gives_180(self):
        vals = [_CH_MID] * 10
        for i in range(4, 9):
            vals[i] = _CH_MAX
        rc = self._make_receiver(vals)
        for angle in rc.get_arm_channels():
            assert angle == 180.0
