"""
test_servo.py – Unit tests for the ServoController.
"""

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "../arm"))

from servo import ServoController, _angle_to_duty, _PULSE_MIN, _PULSE_MAX, _PERIOD_US


class TestAngleToDuty:
    def test_zero_degrees(self):
        duty = _angle_to_duty(0)
        expected = int(_PULSE_MIN / _PERIOD_US * 65535)
        assert duty == expected

    def test_180_degrees(self):
        duty = _angle_to_duty(180)
        expected = int(_PULSE_MAX / _PERIOD_US * 65535)
        assert duty == expected

    def test_90_degrees_is_midpoint(self):
        d0   = _angle_to_duty(0)
        d180 = _angle_to_duty(180)
        d90  = _angle_to_duty(90)
        assert d0 < d90 < d180

    def test_clamp_above_180(self):
        assert _angle_to_duty(270) == _angle_to_duty(180)

    def test_clamp_below_zero(self):
        assert _angle_to_duty(-45) == _angle_to_duty(0)


class TestServoController:
    def _make_controller(self, n=6):
        """Create a ServoController backed by mock PWM objects."""
        class MockPWM:
            def __init__(self, pin):
                self.duty = None
            def freq(self, f):
                pass
            def duty_u16(self, d):
                self.duty = d

        class MockPin:
            def __init__(self, p):
                self.p = p

        # Monkey-patch inside servo module
        import servo as servo_mod
        original_pwm = servo_mod.PWM
        original_pin = servo_mod.Pin
        servo_mod.PWM = MockPWM
        servo_mod.Pin = MockPin
        ctrl = ServoController(list(range(n)))
        servo_mod.PWM = original_pwm
        servo_mod.Pin = original_pin
        return ctrl

    def test_initial_angles_zero(self):
        ctrl = self._make_controller()
        for i in range(6):
            assert ctrl.get_angle(i) == 0.0

    def test_set_angle_updates_stored(self):
        ctrl = self._make_controller()
        ctrl.set_angle(0, 90.0)
        assert ctrl.get_angle(0) == 90.0

    def test_out_of_range_index_ignored(self):
        ctrl = self._make_controller(6)
        ctrl.set_angle(99, 90.0)   # should not raise

    def test_hold_does_not_change_angles(self):
        ctrl = self._make_controller()
        ctrl.set_angle(2, 45.0)
        ctrl.hold()
        assert ctrl.get_angle(2) == 45.0
