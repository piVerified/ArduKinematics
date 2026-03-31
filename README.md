# ArduKinematics
**A Modular, High-Performance Inverse Kinematics Library for ArduPilot**

ArduKinematics is a lightweight Lua-based framework designed to provide standardized manipulator control for the ArduPilot ecosystem. By abstracting complex trigonometric and iterative solvers, it enables N-DOF serial manipulators (humanoid limbs, rover arms) to be commanded via simple Cartesian $(x, y, z)$ coordinates.

## Key Features

* **Heuristic Solver Engine:** Implementation of the **FABRIK** (Forward And Backward Reaching Inverse Kinematics) algorithm, optimized for low-latency execution on flight controller hardware.
* **Coordinate Frame Standardization:** Built on **Denavit-Hartenberg (DH) Parameters**, allowing the library to be reconfigured for any arm geometry via a simple config table.
* **pi.FO (Field Operator) Reference:** A 5-DOF high-fidelity simulation model used as the "Golden Standard" for library validation.
* **"Fusion-to-Flight" Pipeline:** A documented workflow for converting Fusion 360 CAD designs into Gazebo Harmonic-ready SDF models.
* **Stability Aware:** Built-in hooks for EKF3 tilt-compensation to protect the vehicle's Center of Gravity (CoG) during arm extension.
* [Fusion Demo](https://youtu.be/Iaz9FXaadHE)
---

## Installation & Setup

### 1. Prerequisites
* **ArduPilot SITL** (Ubuntu 22.04 / WSL2 recommended)
* **Gazebo Harmonic**
* **ArduPilot-Gazebo Plugin**

### 2. Environment Configuration
Add the resource path to your `~/.bashrc`:
```bash
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:~/ArduKinematics/simulation/models
```

### 3. Launching the SITL Environment
**Terminal 1 (Gazebo):**
```bash
gz sim -v4 -r pifo_test.world
```
**Terminal 2 (ArduPilot):**
```bash
sim_vehicle.py -v Rover -f gazebo-pifo --model gazebo-pifo -I0
```

---

## Usage Example (Lua)

Moving a 2-DOF arm is reduced to a single API call:

```lua
local kinematics = require('ArduKinematics_Core')

-- Define target in meters (x, y, z)
local target = Vector3f(0.2, 0.0, 0.15) 

-- The library handles the FABRIK iterations and joint limits automatically
local success = kinematics.move_to_target(target)

if success then
    gcs:send_text(6, "ArduKinematics: Target reached")
else
    gcs:send_text(4, "ArduKinematics: Target out of reach!")
end
```

---

## Project Roadmap

* [x] Mathematical Foundation (DH-Parameters & Primer)
* [x] Current Phase: URDF/SDF Model Finalization (Gazebo Harmonic)
* [x] Core FABRIK Solver Implementation (Lua)
* [x] MAVLink Telemetry Integration (`NAMED_VALUE_FLOAT`)
