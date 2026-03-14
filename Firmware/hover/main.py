"""
main.py – pi.FO hover (flight controller) firmware entry point.

Runs on the Hover MCU (e.g. Raspberry Pi Pico W / ESP32).

Responsibilities
----------------
* Read RC PWM channels from the RC receiver.
* Run a simple attitude-stabilisation loop (roll / pitch / yaw / throttle).
* Forward arm-command messages received from the ground station over a
  secondary UART to the Arm MCU using the shared arm_protocol framing.
"""

import time

# MicroPython machine / uasyncio imports are available on target hardware.
# On a desktop Python environment they can be substituted for testing.
try:
    from machine import UART, Pin, PWM
    import uasyncio as asyncio
except ImportError:  # running unit tests on CPython
    from unittest.mock import MagicMock
    UART = PWM = Pin = MagicMock
    import asyncio

from imu import IMU
from esc import ESCController
from rc_receiver import RCReceiver
import sys
sys.path.insert(0, "../shared")
from arm_protocol import encode_set_joints, encode_stop, encode_home

# ---------------------------------------------------------------------------
# Hardware pin configuration (adjust to match your wiring)
# ---------------------------------------------------------------------------
RC_UART_ID   = 0   # UART0 – receives SBUS / IBUS from RC receiver
ARM_UART_ID  = 1   # UART1 – sends arm commands to Arm MCU
BAUD_RC      = 100_000
BAUD_ARM     = 115_200

ESC_PINS = [2, 3, 4, 5]  # GP2-GP5: motor ESC signal lines (quad layout)

# ---------------------------------------------------------------------------
# Control loop frequency
# ---------------------------------------------------------------------------
LOOP_HZ  = 400       # attitude loop rate (Hz)
LOOP_US  = 1_000_000 // LOOP_HZ


def setup():
    """Initialise all peripherals."""
    rc_uart  = UART(RC_UART_ID,  baudrate=BAUD_RC)
    arm_uart = UART(ARM_UART_ID, baudrate=BAUD_ARM)
    imu      = IMU()                     # see imu.py
    escs     = ESCController(ESC_PINS)   # see esc.py
    rc       = RCReceiver(rc_uart)       # see rc_receiver.py
    return rc_uart, arm_uart, imu, escs, rc


async def attitude_loop(imu, escs, rc):
    """
    Very simplified attitude stabilisation (placeholder for a full PID stack).

    A production implementation would use a proper PID controller (e.g.
    Betaflight / ArduPilot algorithms).  This loop exists to show the
    integration structure expected by pi.FO.
    """
    while True:
        roll, pitch, yaw = imu.get_angles()
        throttle, roll_sp, pitch_sp, yaw_sp = rc.get_channels()

        # Compute motor outputs (simplified – replace with real PID)
        m_fr = throttle - roll_sp + pitch_sp - yaw_sp   # front-right (CW)
        m_fl = throttle + roll_sp + pitch_sp + yaw_sp   # front-left  (CCW)
        m_rr = throttle - roll_sp - pitch_sp + yaw_sp   # rear-right  (CCW)
        m_rl = throttle + roll_sp - pitch_sp - yaw_sp   # rear-left   (CW)

        # Saturate: shift all outputs up/down to keep every motor in [0, 1]
        outputs = [m_fr, m_fl, m_rr, m_rl]
        deficit = min(0.0, min(outputs))
        excess  = max(0.0, max(outputs) - 1.0)
        outputs = [o - deficit - excess for o in outputs]

        escs.set_throttles(*outputs)
        await asyncio.sleep_ms(LOOP_US // 1_000)


async def arm_forward_loop(rc, arm_uart):
    """
    Read arm-pose channels from the RC transmitter (channels 5-9) and
    forward encoded arm_protocol frames to the Arm MCU.
    """
    while True:
        joints = rc.get_arm_channels()   # returns list of 5 angles in degrees
        gripper = rc.get_gripper()       # returns 0.0 – 1.0
        frame = encode_set_joints(joints, gripper)
        arm_uart.write(frame)
        await asyncio.sleep_ms(20)   # 50 Hz arm update rate


async def main():
    rc_uart, arm_uart, imu, escs, rc = setup()
    await asyncio.gather(
        attitude_loop(imu, escs, rc),
        arm_forward_loop(rc, arm_uart),
    )


if __name__ == "__main__":
    asyncio.run(main())
