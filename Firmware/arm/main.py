"""
main.py – pi.FO 5-DOF arm controller firmware entry point.

Runs on the Arm MCU (e.g. Raspberry Pi Pico / ESP32).

Responsibilities
----------------
* Listen for arm_protocol frames arriving over UART from the Hover MCU.
* Decode frames and drive 5 servo motors (J1-J5) and the gripper servo.
* Enforce per-joint angle limits to protect the arm structure.
"""

try:
    from machine import UART, Pin
    import uasyncio as asyncio
except ImportError:
    from unittest.mock import MagicMock
    UART = Pin = MagicMock
    import asyncio

import sys
sys.path.insert(0, "../shared")
from arm_protocol import (
    decode_frame,
    CMD_SET_JOINTS, CMD_STOP, CMD_HOME, CMD_GRIPPER,
    JOINT_BASE, JOINT_SHOULDER, JOINT_ELBOW, JOINT_WRIST_PITCH,
    JOINT_WRIST_ROLL, JOINT_GRIPPER,
)
from servo import ServoController

# ---------------------------------------------------------------------------
# Hardware pin configuration (adjust to match your wiring)
# ---------------------------------------------------------------------------
ARM_UART_ID = 0          # UART0 receives commands from the Hover MCU
BAUD_ARM    = 115_200

# GPIO pins for each servo (J1 … J5 + gripper)
SERVO_PINS = [10, 11, 12, 13, 14, 15]

# ---------------------------------------------------------------------------
# Home (stow) position for each joint in degrees
# ---------------------------------------------------------------------------
HOME_ANGLES = [90, 45, 135, 90, 90, 0]   # [J1, J2, J3, J4, J5, gripper]

# Per-joint angle limits [min_deg, max_deg]
JOINT_LIMITS = [
    (0,   180),   # J1 Base
    (15,  165),   # J2 Shoulder  (structural limits)
    (15,  165),   # J3 Elbow
    (0,   180),   # J4 Wrist Pitch
    (0,   180),   # J5 Wrist Roll
    (0,   100),   # Gripper (0 = closed, 100 = open %)
]


def _clamp(value: float, min_val: float, max_val: float) -> float:
    return max(min_val, min(max_val, value))


def setup():
    uart   = UART(ARM_UART_ID, baudrate=BAUD_ARM)
    servos = ServoController(SERVO_PINS)
    return uart, servos


async def command_loop(uart, servos):
    """Read incoming UART bytes and dispatch decoded arm_protocol commands."""
    buf = bytearray()

    while True:
        # Drain available bytes
        if hasattr(uart, 'any') and uart.any():
            buf += uart.read(uart.any())

        # Try to decode a frame
        if len(buf) >= 4:
            result = decode_frame(bytes(buf))
            if result is not None:
                cmd_id, payload = result
                _dispatch(cmd_id, payload, servos)
                # Consume the frame (header + cmd + len + payload + checksum)
                frame_len = 3 + len(payload) + 1
                buf = buf[frame_len:]
            else:
                # Discard the first byte and try again
                buf = buf[1:]

        await asyncio.sleep_ms(1)


def _dispatch(cmd_id: int, payload: bytes, servos):
    """Apply a decoded command to the servo controller."""
    if cmd_id == CMD_SET_JOINTS:
        # payload: [J1, J2, J3, J4, J5, gripper_pct_0-100]
        for i, limit in enumerate(JOINT_LIMITS):
            if i < len(payload):
                angle = _clamp(payload[i], limit[0], limit[1])
                servos.set_angle(i, angle)

    elif cmd_id == CMD_STOP:
        servos.hold()

    elif cmd_id == CMD_HOME:
        for i, angle in enumerate(HOME_ANGLES):
            servos.set_angle(i, angle)

    elif cmd_id == CMD_GRIPPER:
        if payload:
            pct = _clamp(payload[0], 0, 100)
            servos.set_angle(JOINT_GRIPPER, pct)


async def main():
    uart, servos = setup()
    # Move to home position on startup
    for i, angle in enumerate(HOME_ANGLES):
        servos.set_angle(i, angle)

    await asyncio.gather(
        command_loop(uart, servos),
    )


if __name__ == "__main__":
    asyncio.run(main())
