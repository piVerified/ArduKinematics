# pi.FO Journal

You can find the original pi.plug journal at [Scrap Book](https://scrapbook.hackclub.com/pi.obj)

---

## pi.FO – Hover + 5-DOF Arm

pi.FO is an aerial manipulation platform combining a quadcopter hover with a
5-degree-of-freedom (5-DOF) serial-link robotic arm.  The project is proposed
as a GSOC contribution.

### Architecture decisions

* **Two-MCU design** – The hover flight controller and the arm controller run
  on separate microcontrollers connected via UART to keep real-time flight
  control completely isolated from arm kinematics.

* **arm_protocol framing** – A simple binary protocol (start byte + cmd ID +
  length-prefixed payload + XOR checksum) is used for reliable, low-overhead
  communication between the two MCUs.

* **iBUS RC input** – Ten RC channels are decoded: CH1-CH4 for flight
  (roll/pitch/throttle/yaw) and CH5-CH10 for arm joints J1-J5 and the
  gripper.

* **50 Hz servo update rate** – The arm controller refreshes all servo
  positions at 50 Hz, matching the standard hobby servo PWM period.

### 5-DOF joint table

| Joint | Axis | Min (°) | Max (°) |
| :---: | :--: | :-----: | :-----: |
| J1 Base | Yaw | 0 | 180 |
| J2 Shoulder | Pitch | 15 | 165 |
| J3 Elbow | Pitch | 15 | 165 |
| J4 Wrist Pitch | Pitch | 0 | 180 |
| J5 Wrist Roll | Roll | 0 | 180 |

---
