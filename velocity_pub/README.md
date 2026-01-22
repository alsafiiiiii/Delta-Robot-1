# High-Performance 3DOF Delta Robot Control System

This workspace contains the complete software stack for controlling a 3DOF Delta Robot with industrial-grade smoothness and precision using standard hobby servos.

## System Architecture

The system operates on a "Brain-Bridge-Muscle" architecture designed to maximize performance and minimize jitter.

### 1. The Brain: `delta_3dof_controller.py`

- **Role**: High-level Motion Planner.
- **Physics**: Generates **Quintic (5th-order) Trajectories** for every move. This ensures Position, Velocity, and Acceleration are continuous (No Jerk).
- **Update Rate**: **100Hz**.
- **Output**: Calculates Inverse Kinematics (IK) for every 10ms timestep and publishes Joint Angles.

### 2. The Bridge: `robot_control.py`

- **Role**: Communication Gateway.
- **Protocol**: Serial at **115200 baud**.
- **Optimization**: Uses a **0.5µs Change Threshold**. It only sends data if the requested move is physically significant, but with enough resolution to utilize the full capability of the servos.

### 3. The Muscle: ESP32 Firmware (`src/servo/`)

- **Role**: Real-time Motor Drive.
- **PWM Frequency**: **200Hz** (5ms latency). Standard servos use 50Hz; we overdrive them to 200Hz for superior responsiveness.
- **Resolution**: **1µs** (High precision).
- **Smoothing**: **0.15** Exponential Moving Average. This filters out electrical noise from 8-bit potentiometers while letting the smooth Python trajectory pass through.

---

## How to Run

### 1. Flash the Firmware

Connect your ESP32 and flash the optimized code:

```bash
idf.py -p /dev/ttyUSB0 flash monitor
```

### 2. Start the Control Loop

Launch the ROS 2 nodes:

```bash
# Terminal 1: The Controller
python3 src/velocity_pub/scripts/delta_3dof_controller.py

# Terminal 2: The Bridge (sends data to ESP32)
python3 src/velocity_pub/scripts/robot_control.py
```

### 3. Send Commands

You can control the robot via G-code or ROS topics.

**Option A: G-Code Script (Recommended)**

```bash
# Run a square test pattern
python3 src/velocity_pub/scripts/delta_gcode_interpreter.py src/velocity_pub/scripts/square_test.gcode
```

*Note: Use `F1`-`F5` for testing. `F15` is extremely fast (0.25m/s).*

**Option B: Direct Target**

```bash
ros2 topic pub --once /delta/target_pose geometry_msgs/msg/Pose "{position: {x: 0.05, y: 0.05, z: -0.25}}"
```

---

## Verification Tools

We have included tools to mathematically prove the system performance.

### 1. Live Plotter

Visualizes the PWM signals and velocity in real-time.

```bash
python3 src/velocity_pub/scripts/live_plotter.py
```

- **Blue Line**: PWM Pulse Width (us).
- **Red Line**: Velocity (deg/s). **Look for a smooth Bell Curve.**

### 2. Motion Verifier

Records a move and generates a statistical report proving zero-velocity start/stop.

```bash
python3 src/velocity_pub/scripts/verify_motion_profile.py
```

*Example Output:*

```text
✅ SUCCESS: Motion starts and ends at rest.
Max Velocity: 0.1049 rad/s
Max Accel:    1.1412 rad/s^2
```

## Physics & Dynamics

The `delta_dynamics.py` module contains the full Inverse Dynamics equations (Lagrange Multipliers) for the robot. It is currently used for theoretical validation but can be enabled for torque-feedforward if you upgrade to torque-controlled motors in the future.
