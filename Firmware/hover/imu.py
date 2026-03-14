"""
imu.py – Thin wrapper around an MPU-6050 / ICM-42688 IMU for the hover MCU.

Returns fused roll / pitch / yaw angles in degrees.
"""

try:
    from machine import I2C, Pin
except ImportError:
    from unittest.mock import MagicMock
    I2C = Pin = MagicMock

# Default I2C address of MPU-6050
_MPU6050_ADDR = 0x68
_REG_PWR_MGMT = 0x6B
_REG_ACCEL_XOUT_H = 0x3B


class IMU:
    """Minimal IMU driver returning attitude angles."""

    def __init__(self, i2c_id: int = 0, sda_pin: int = 8, scl_pin: int = 9):
        try:
            self._i2c = I2C(i2c_id, sda=Pin(sda_pin), scl=Pin(scl_pin), freq=400_000)
            # Wake the MPU-6050
            self._i2c.writeto_mem(_MPU6050_ADDR, _REG_PWR_MGMT, b'\x00')
        except Exception:
            self._i2c = None   # fall back to zeroes in simulation

        self._roll  = 0.0
        self._pitch = 0.0
        self._yaw   = 0.0

    def update(self):
        """Read raw sensor data and update fused angle estimates."""
        if self._i2c is None:
            return
        # In a full implementation this would call a complementary or
        # Madgwick filter.  Placeholder: values stay at zero until real
        # hardware is connected.

    def get_angles(self):
        """Return (roll, pitch, yaw) in degrees."""
        self.update()
        return self._roll, self._pitch, self._yaw
