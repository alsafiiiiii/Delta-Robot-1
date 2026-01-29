# Delta Robot Control Package (velocity_pub)

ROS 2 package containing all control nodes, G-code interpreters, and utilities for the Delta Robot.

## System Architecture

```
G-Code / GUI / Joystick
        │
        ▼
┌──────────────────────────┐
│  delta_3dof_controller   │  ← IK solver (using delta_ik.py)
│  /delta/target_pose      │  ← Direct pose input
│  /delta/cartesian_traj   │  ← Time-encoded trajectory
└───────────┬──────────────┘
            │
            ▼
┌──────────────────────────┐
│     robot_control.py     │  ← Serial @ 115200 baud
│  Format: T0:deg D:ms     │
└───────────┬──────────────┘
            │
            ▼
        ESP32 (50Hz)
```

## Key Scripts

### Core Control

| Script | Description |
|--------|-------------|
| `delta_3dof_controller.py` | Main controller node - handles IK and trajectory |
| `delta_ik.py` | Pure Python inverse kinematics (no external deps) |
| `robot_control.py` | Serial bridge to ESP32 (real hardware) |
| `sim_control.py` | Gazebo bridge with ESP32-like interpolation |

### Input Sources

| Script | Description |
|--------|-------------|
| `delta_gcode_interpreter.py` | G-code file executor |
| `gui_controller.py` | Interactive GUI with sliders |
| `joystick_controller.py` | Gamepad/joystick control |
| `camera_pnp.py` | Vision-based pick and place |

### Utilities

| Script | Description |
|--------|-------------|
| `quintic_trajectory.py` | Smooth trajectory generator |
| `live_plotter.py` | Real-time motion visualization |

## Usage

### Start the Controller

```bash
# Terminal 1: IK Controller
python3 delta_3dof_controller.py

# Terminal 2: Serial Bridge
python3 robot_control.py
```

### Run G-Code

```bash
python3 delta_gcode_interpreter.py square_test.gcode
```

### GUI Control

```bash
python3 gui_controller.py
```

## G-Code Format

```gcode
G21          ; Units: mm
G90          ; Absolute positioning
F200         ; Speed: 200 mm/s
G1 X50 Y50 Z-220
G1 X-50 Y50 Z-220
G28          ; Home
```

## Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/delta/target_pose` | `Pose` | Direct position command |
| `/delta/cartesian_trajectory` | `JointTrajectory` | Time-encoded path |
| `/model/delta_robot/joint_trajectory` | `JointTrajectory` | Output to simulation/hardware |

## IK Module (delta_ik.py)

Pure Python implementation matching `visual_kinematics` output.

```python
from delta_ik import DeltaIK

ik = DeltaIK()
t1, t2, t3 = ik.inverse(x, y, z)      # Returns radians
t1, t2, t3 = ik.inverse_deg(x, y, z)  # Returns degrees
```

**Accuracy**: Matches `visual_kinematics` to 15 decimal places (64-bit float precision).
