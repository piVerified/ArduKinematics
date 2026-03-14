# pi.FO

the Field Operator is an mobile manupulator, it has an fully animated arm with 5 DOF! 

---
# Features

1. 5-DOF Kinematic Integration: A fully articulated robotic arm featuring 5 Degrees of Freedom (Base, Shoulder, Elbow, Wrist Pitch, and Gripper). Integrated via Inverse Kinematics (IK) to allow coordinate-based movement rather than simple servo-angle commands.

2. Unified Rover-Arm Control Profile: A custom ArduPilot vehicle configuration that seamlessly bridges the gap between Rover navigation (wheel drive) and Manipulator operation (arm control), allowing for complex "Drive-to-Pick" missions.

3. High-Fidelity Physics Simulation: Validated through ArduPilot SITL and Gazebo Harmonic. The arm features realistic inertia tensors and collision physics, ensuring that maneuvers in the simulator translate accurately to the physical hardware.

4. Dual-Mode Operation (Auto/Manual): Support for manual teleoperation via MAVLink-compatible controllers and autonomous "Work Modes" where the arm can perform pre-programmed sequences based on GPS or sensor triggers.
