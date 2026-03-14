"""
esc.py – ESC (Electronic Speed Controller) PWM driver for pi.FO hover.

Sends 50 Hz PWM signals (1000–2000 µs pulse width) to up to 4 ESCs.
"""

try:
    from machine import Pin, PWM
except ImportError:
    from unittest.mock import MagicMock
    Pin = PWM = MagicMock

_PWM_FREQ  = 50           # Hz  (standard ESC protocol)
_PULSE_MIN = 1_000        # µs  (motor stopped)
_PULSE_MAX = 2_000        # µs  (full throttle)
_PERIOD_US = 1_000_000 // _PWM_FREQ


def _throttle_to_duty(throttle_pct: float) -> int:
    """Convert 0.0–1.0 throttle to PWM duty cycle (16-bit)."""
    throttle_pct = max(0.0, min(1.0, throttle_pct))
    pulse_us = _PULSE_MIN + int(throttle_pct * (_PULSE_MAX - _PULSE_MIN))
    # MicroPython PWM duty_u16 range: 0–65535
    return int(pulse_us / _PERIOD_US * 65535)


class ESCController:
    """Drive up to 4 ESCs via hardware PWM."""

    def __init__(self, pins: list):
        self._escs = []
        for p in pins:
            try:
                pwm = PWM(Pin(p))
                pwm.freq(_PWM_FREQ)
                pwm.duty_u16(0)
                self._escs.append(pwm)
            except Exception:
                self._escs.append(None)

    def arm(self):
        """Send minimum throttle to all ESCs to arm them."""
        for esc in self._escs:
            if esc:
                esc.duty_u16(_throttle_to_duty(0.0))

    def set_throttles(self, *throttles):
        """
        Set individual motor throttles.

        Parameters
        ----------
        *throttles : float
            Throttle values in range 0.0–1.0, one per ESC.
        """
        for esc, t in zip(self._escs, throttles):
            if esc:
                esc.duty_u16(_throttle_to_duty(t))

    def stop(self):
        """Cut all motors immediately."""
        for esc in self._escs:
            if esc:
                esc.duty_u16(0)
