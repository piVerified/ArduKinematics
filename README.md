# pi.FO

meet **pi.FO**, an autonomous hover (multirotor UAV) with an integrated **5-DOF robotic arm** — built for aerial manipulation tasks. This project is a GSOC proposal combining drone flight control with a lightweight 5-degree-of-freedom manipulator arm.

---
# Overview

pi.FO pairs a quadcopter-class hover platform with a custom 5-DOF serial-link arm to enable tasks such as:

* Aerial pick-and-place
* Remote inspection and contact sensing
* Payload delivery with precision placement

The hover and arm firmware run on separate microcontrollers and communicate over a shared serial bus, allowing independent development and easy extension.

---
# 5-DOF Arm Degrees of Freedom

| Joint | Motion | Actuator |
| :---: | :----: | :------: |
| J1 – Base | Yaw rotation | Servo |
| J2 – Shoulder | Pitch (up/down) | Servo |
| J3 – Elbow | Pitch (up/down) | Servo |
| J4 – Wrist Pitch | Pitch (tilt) | Servo |
| J5 – Wrist Roll | Roll (rotate) | Servo |

An end-effector (gripper) is mounted after J5 and driven by a sixth servo (not counted as a DOF).

---
# Architecture

```
┌──────────────────────────────────┐
│         Ground Station / RC      │
└────────────────┬─────────────────┘
                 │ MAVLink / RC PWM
     ┌───────────▼────────────┐
     │   Flight Controller    │  (Hover MCU)
     │  (hover firmware)      │
     └───────────┬────────────┘
                 │ UART (arm_protocol)
     ┌───────────▼────────────┐
     │   Arm Controller       │  (Arm MCU)
     │  (arm firmware)        │
     └───────────┬────────────┘
                 │ PWM
     ┌───────────▼────────────┐
     │  5×Servo + Gripper     │
     └────────────────────────┘
```

---
# Repository Structure

```
pi.FO/
├── CAD/                  # Fusion 360 / STEP files for frame and arm
├── Firmware/
│   ├── hover/            # Quadcopter flight-control firmware (MicroPython)
│   ├── arm/              # 5-DOF arm servo-control firmware (MicroPython)
│   └── shared/           # Shared arm_protocol message definitions
├── PCB/                  # KiCAD schematics & PCB layouts
├── production/           # Gerber files and BOM
├── Journal archive/      # Development journal
└── assets/               # Images and media
```

---
## Fusion 360 / CAD

| Frame (Top) | Arm Assembly |
| :---------: | :----------: |
| *(coming soon)* | *(coming soon)* |

---
## KiCAD

| Hover Power/ESC Schematic | Arm Driver Schematic |
| :-----------------------: | :------------------: |
| *(coming soon)* | *(coming soon)* |

---
# Softwares Used

* KiCAD → PCB Design
* Fusion 360 → Frame & arm CAD
* MicroPython → Hover and arm firmware

---
# Bibliography

> This project would not have been possible without the help of the open-source drone community, the Hack Club blueprint guide, and a bit of AI.
