"""
rc_receiver.py – IBUS RC receiver parser for pi.FO hover.

Reads an iBUS frame (32 bytes) from the RC receiver over UART and exposes
individual channel values.

Channel mapping (configurable on the transmitter)
--------------------------------------------------
CH1 – Roll       (aileron)
CH2 – Pitch      (elevator)
CH3 – Throttle
CH4 – Yaw        (rudder)
CH5 – J1 Base    (arm joint 1 angle)
CH6 – J2 Shoulder
CH7 – J3 Elbow
CH8 – J4 Wrist Pitch
CH9 – J5 Wrist Roll
CH10 – Gripper   (open/close)
"""

_IBUS_FRAME_LEN  = 32
_IBUS_HEADER     = b'\x20\x40'
_CH_MIN          = 1000   # µs
_CH_MID          = 1500
_CH_MAX          = 2000


def _normalise(raw: int) -> float:
    """Map raw iBUS value (1000–2000) to -1.0 … +1.0."""
    return (max(_CH_MIN, min(_CH_MAX, raw)) - _CH_MID) / 500.0


def _normalise_pos(raw: int) -> float:
    """Map raw iBUS value (1000–2000) to 0.0 … 1.0."""
    return (max(_CH_MIN, min(_CH_MAX, raw)) - _CH_MIN) / 1000.0


def _raw_to_angle(raw: int, max_deg: float = 180.0) -> float:
    """Map raw iBUS value (1000–2000) to 0 … max_deg."""
    return _normalise_pos(raw) * max_deg


class RCReceiver:
    """Parse iBUS frames from a UART-connected RC receiver."""

    NUM_CHANNELS = 10

    def __init__(self, uart):
        self._uart     = uart
        self._channels = [_CH_MID] * self.NUM_CHANNELS
        self._buf      = bytearray()

    # ------------------------------------------------------------------
    # Internal parsing
    # ------------------------------------------------------------------

    def _read_uart(self):
        """Drain any available bytes from the UART into the internal buffer."""
        if self._uart and hasattr(self._uart, 'read'):
            available = self._uart.any() if hasattr(self._uart, 'any') else 0
            if available:
                self._buf += self._uart.read(available)

    def _parse(self):
        """Find and parse the latest complete iBUS frame in the buffer."""
        while True:
            idx = self._buf.find(_IBUS_HEADER)
            if idx == -1:
                self._buf.clear()
                return
            self._buf = self._buf[idx:]
            if len(self._buf) < _IBUS_FRAME_LEN:
                return
            frame = self._buf[:_IBUS_FRAME_LEN]
            # Verify checksum (sum of first 30 bytes + stored 2-byte checksum = 0xFFFF)
            checksum = 0xFFFF - sum(frame[:30]) & 0xFFFF
            stored = frame[30] | (frame[31] << 8)
            if checksum == stored:
                for i in range(self.NUM_CHANNELS):
                    offset = 2 + i * 2
                    self._channels[i] = frame[offset] | (frame[offset + 1] << 8)
            self._buf = self._buf[_IBUS_FRAME_LEN:]

    def _refresh(self):
        self._read_uart()
        self._parse()

    # ------------------------------------------------------------------
    # Public API used by main.py
    # ------------------------------------------------------------------

    def get_channels(self):
        """Return (throttle 0-1, roll -1..1, pitch -1..1, yaw -1..1)."""
        self._refresh()
        throttle = _normalise_pos(self._channels[2])
        roll     = _normalise(self._channels[0])
        pitch    = _normalise(self._channels[1])
        yaw      = _normalise(self._channels[3])
        return throttle, roll, pitch, yaw

    def get_arm_channels(self):
        """Return list of 5 joint angles in degrees [J1 .. J5]."""
        self._refresh()
        return [_raw_to_angle(self._channels[4 + i]) for i in range(5)]

    def get_gripper(self):
        """Return gripper open fraction 0.0 – 1.0."""
        self._refresh()
        return _normalise_pos(self._channels[9])
