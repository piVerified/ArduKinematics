"""
servo.py – PWM servo driver for the pi.FO 5-DOF arm.

Generates 50 Hz PWM signals with pulse widths between 500 µs (0°) and
2500 µs (180°), compatible with most hobby digital servos.
"""

try:
    from machine import Pin, PWM
except ImportError:
    from unittest.mock import MagicMock
    Pin = PWM = MagicMock

_PWM_FREQ  = 50           # Hz
_PERIOD_US = 1_000_000 // _PWM_FREQ   # 20 000 µs
_PULSE_MIN = 500          # µs → 0°
_PULSE_MAX = 2_500        # µs → 180°


def _angle_to_duty(angle_deg: float) -> int:
    """Convert an angle (0–180°) to a 16-bit PWM duty value."""
    angle_deg = max(0.0, min(180.0, float(angle_deg)))
    pulse_us  = _PULSE_MIN + int(angle_deg / 180.0 * (_PULSE_MAX - _PULSE_MIN))
    return int(pulse_us / _PERIOD_US * 65535)


class ServoController:
    """Drive multiple servos via hardware PWM channels."""

    def __init__(self, pins: list):
        self._servos = []
        for p in pins:
            try:
                pwm = PWM(Pin(p))
                pwm.freq(_PWM_FREQ)
                self._servos.append(pwm)
            except Exception:
                self._servos.append(None)
        self._angles = [0.0] * len(self._servos)

    def set_angle(self, index: int, angle_deg: float):
        """
        Move servo *index* to *angle_deg* (0–180°).

        For the gripper channel the value represents an open percentage
        (0–100 %) which is mapped linearly to the servo range.
        """
        if index < 0 or index >= len(self._servos):
            return
        servo = self._servos[index]
        if servo:
            servo.duty_u16(_angle_to_duty(angle_deg))
        self._angles[index] = angle_deg

    def hold(self):
        """Re-assert all current positions (no movement)."""
        for i, angle in enumerate(self._angles):
            self.set_angle(i, angle)

    def get_angle(self, index: int) -> float:
        """Return the last commanded angle for servo *index*."""
        if 0 <= index < len(self._angles):
            return self._angles[index]
        return 0.0
