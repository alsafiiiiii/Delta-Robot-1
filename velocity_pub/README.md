# Delta Robot Control Package (velocity_pub)

This package contains the core control logic, trajectory generators, and G-code interpreters for the Delta Robot.

## System Architecture

The control pipeline follows this flow:
1. **Input Sources**: GUI (PyQt), G-Code Interpreter, or Joystick.
2. **Trajectory Generator**: Computes smooth quintic or linear paths in Cartesian space.
3. **Inverse Kinematics**: Translates X/Y/Z targets into joint angles (radians).
4. **Hardware Bridge**: Sends these angles to the real motors via Serial data.

## Hardware Support
This project is designed for **Feetech Serial Bus Servos**:
- **3x ST3215-HS** (Main Arm/Bicep)
- **2x STS3032** (End-Effector Differential)

Commands are sent as high-speed serial data packets (921,600 baud), **not PWM**.

## Key Scripts

- `delta_3dof_controller.py`: Main controller node handling IK and trajectory.
- `delta_ik.py`: High-precision Python inverse kinematics solver.
- `delta_gcode_interpreter.py`: Executes G-code sequences from the `gcode/` folder.
- `quintic_trajectory.py`: Generates smooth velocity-profiled movements.

## Usage

### 1. Launch IK Controller
```bash
ros2 run velocity_pub delta_3dof_controller.py
```

### 2. Execute G-Code
```bash
ros2 run velocity_pub delta_gcode_interpreter.py src/velocity_pub/gcode/square_test.gcode
```

## Topics

| Topic | Message Type | Purpose |
|-------|--------------|---------|
| `/delta/target_pose` | `geometry_msgs/Pose` | Direct Cartesian target input |
| `/delta/joint_commands` | `trajectory_msgs/JointTrajectory` | Calculated joint angles (radians) |
| `/joint_states` | `sensor_msgs/JointState` | Feedback from simulation/hardware |

---
