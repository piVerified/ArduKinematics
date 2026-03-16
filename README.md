# ArduKinematics Ardupilot Library

ArduKinematics is a modular, lightweight Inverse Kinematics (IK) library for ArduPilot, designed to provide standardized manipulator control via Lua scripting. pi.FO (Field Operator) serves as the high-fidelity reference hardware and simulation model for this framework.

## Key Features
1. Standardized IK Library (ArduKinematics): A reusable Lua framework for $N$-DOF serial manipulators. It allows users to input link lengths and target $(x, y, z)$ coordinates, abstracting away complex trigonometry from the end-user.
2. pi.FO Reference Arm: A 5-DOF manipulator (Base, Shoulder, Elbow, Wrist, and Gripper) used to validate the library. Featuring link lengths of $230\text{mm}$ and $200\text{mm}$, it provides a real-world testbed for coordinate-based movement.
3. "Fusion-to-Flight" Pipeline: A documented workflow for converting Fusion 360 CAD designs into Gazebo Harmonic-ready SDF models via Xacro and URDF.
4. SITL Validation: Fully integrated with ArduPilot SITL and Gazebo Harmonic, featuring realistic inertia tensors and joint limit constraints validated through the ArduPilot-Gazebo plugin.


# Getting Started

### 1. Prerequisites

Before running the simulation, ensure you have the following installed:

* **ArduPilot SITL**: Follow the [standard installation guide](https://ardupilot.org/dev/docs/setting-up-sitl-on-linux.html).
* **Gazebo Harmonic**: The latest physics engine for high-fidelity simulation.
* **ArduPilot-Gazebo Plugin**: Essential for bridging Lua commands to the simulator.

### 2. Installation & Setup

Clone the repository and point Gazebo to your custom model folder by adding this to your `~/.bashrc`:

```bash
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:~/pi.FO/simulation/models

```

### 3. Running the Simulation

1. **Launch Gazebo**:
```bash
gz sim -v4 -r pifo_test.world

```


2. **Launch ArduPilot SITL** (in a new terminal):
```bash
cd ~/ardupilot/ArduRover
sim_vehicle.py -v Rover -f gazebo-pifo --model gazebo-pifo -I0

```



### 4. Basic Library Usage (Lua)

Once the simulation is running, your script can move the arm by simply passing Cartesian coordinates to the library:

```lua
-- Example: Moving the pi.FO arm to a specific point
local kinematics = require('AP_kinematics_core')

-- Define target in meters (x, y, z)
local target = Vector3f(0.2, 0.0, 0.15) 

-- The library handles the Geometric/FABRIK math automatically
local success = kinematics.move_to_target(target)

if success then
    gcs:send_text(6, "pi.FO: Target reached successfully")
else
    gcs:send_text(4, "pi.FO: Target out of reach!")
end

```

---

## Rad map
- [ ] AP_kinematics_core.lua
- [ ] AP_kinematics_utils.lua
- [ ] pi.fo_Example.lua
- [ ] CAD to Gazibo conversion guide
