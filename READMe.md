# Delta Robot Control System

A high-performance 3DOF Delta Robot control system using ROS 2 and ESP32.

## Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                        INPUT SOURCES                             │
├───────────────┬─────────────────┬───────────────────────────────┤
│  GUI/Joystick │   G-Code Files  │        Camera PNP             │
└───────┬───────┴────────┬────────┴──────────┬────────────────────┘
        │                │                   │
        ▼                ▼                   ▼
┌─────────────────────────────────────────────────────────────────┐
│                   delta_3dof_controller.py                       │
│  • Pure Python IK (delta_ik.py)                                  │
│  • Dual mode: Direct Pose + Time-Encoded Trajectory              │
└───────────────────────────────┬─────────────────────────────────┘
                                │
                                ▼
┌─────────────────────────────────────────────────────────────────┐
│                      robot_control.py                            │
│  • Serial bridge at 115200 baud                                  │
│  • Protocol: T0:deg D:duration_ms                                │
└───────────────────────────────┬─────────────────────────────────┘
                                │
                                ▼
┌─────────────────────────────────────────────────────────────────┐
│                   ESP32 Firmware (servo/)                        │
│  • 50Hz control loop with hybrid interpolation                   │
│  • MCPWM-based PWM generation                                    │
└─────────────────────────────────────────────────────────────────┘
```

## Packages

| Package | Description |
|---------|-------------|
| `velocity_pub` | ROS 2 nodes for robot control, G-code interpretation, GUI |
| `servo` | ESP32 firmware (ESP-IDF) for servo control |
| `delta_robot_description` | URDF/SDF models and meshes |
| `delta_robot_sim` | Gazebo simulation launch files |
| `archive` | Legacy scripts and firmware versions |

## Quick Start

### 1. Flash ESP32 Firmware

```bash
cd src/servo
idf.py build flash monitor
```

### 2. Start ROS 2 Nodes

```bash
# Terminal 1: Controller (IK solver)
python3 src/velocity_pub/scripts/delta_3dof_controller.py

# Terminal 2: Serial bridge
python3 src/velocity_pub/scripts/robot_control.py
```

### 3. Run G-Code

```bash
python3 src/velocity_pub/scripts/delta_gcode_interpreter.py src/velocity_pub/scripts/square_test.gcode
```

## Hardware

- **Robot**: 3DOF Linear Delta (104mm base, 40mm end-effector, 105mm upper arm, 205mm lower arm)
- **Servos**: 16kg·cm metal gear digital servos (50Hz PWM)
- **Controller**: ESP32 DevKit V1
- **Communication**: USB Serial at 115200 baud

## Key Files

| File | Purpose |
|------|---------|
| `delta_ik.py` | Pure Python inverse kinematics |
| `delta_3dof_controller.py` | Main ROS 2 controller node |
| `delta_gcode_interpreter.py` | G-code parser and executor |
| `robot_control.py` | Serial bridge to ESP32 |
| `gui_controller.py` | Interactive GUI control |
| `joystick_controller.py` | Gamepad input handler |
