# Delta Robot ROS 2 Project

This repository contains the ROS 2 packages for controlling and simulating a 5-DOF Delta Robot. The project supports both Gazebo simulation and real hardware control via a serial bridge.

## Getting Started

### Prerequisites
- ROS 2 (Humble or later recommended)
- Gazebo Sim (Ignition)
- `gz_ros2_control`
- Python 3 with `pyserial`

### Installation & Build
```bash
# Clone the repository into your workspace src
cd ~/major_project_ws/src
git clone https://github.com/RikisuT/Delta-Robot.git .

# Build the workspace
cd ~/major_project_ws
colcon build --symlink-install

# Source the workspace
source install/setup.bash
```

---

## Running the Robot

### 1. Simulation Mode (Gazebo)
To run the robot in a virtual environment:

1. **Launch Simulation & Controllers:**
   ```bash
   ros2 launch delta_robot_sim delta_robot_spawn.launch.py
   ```
   This will start Gazebo, spawn the robot, and load the joint controllers.

2. **Launch the Control GUI:**
   ```bash
   ros2 launch delta_robot_gui delta_robot_gui.launch.py sim_mode:=true
   ```

---

### 2. Real Hardware Mode
To control the physical robot:

1. **Start the Hardware Bridge:**
   Ensure your motor driver (ESP32) is connected via USB.
   ```bash
   # You may need to grant permissions to the serial port
   sudo chmod 666 /dev/ttyUSB0
   
   # Run the motor control node
   ros2 run delta_hardware_bridge motor_control_node
   ```
   *(Note: Ensure the `motor_control_node` is correctly mapped in `setup.py` or run directly via `python3 src/delta_hardware_bridge/delta_hardware_bridge/motor_control_node.py`)*

2. **Launch the Control GUI:**
   ```bash
   ros2 launch delta_robot_gui delta_robot_gui.launch.py sim_mode:=false
   ```

---

## Project Structure

- **`delta_robot_description`**: URDF/SDF models and meshes.
- **`delta_robot_sim`**: Gazebo world and spawn configurations.
- **`delta_controllers`**: Kinematics and motion control logic.
- **`delta_hardware_bridge`**: Serial communication with physical motors (IDs 1-5).
- **`delta_robot_gui`**: PyQt5 interface for manual and task control.
- **`delta_tasks`**: G-code parsing and task sequencing.

## Hardware Specifications
This robot uses **Serial Bus Servos** (ST-series), not PWM. Commands are sent as serial data packets.

### Motor ID Mapping
- **ID 1, 2, 3**: **Feetech ST3215-HS** (Bicep/Arm Servos)
- **ID 4, 5**: **Feetech STS3032** (End-Effector Differential Servos)

**GitHub Repository:** [https://github.com/RikisuT/Delta-Robot](https://github.com/RikisuT/Delta-Robot)

---

## License Detail
See the [LICENSE](LICENSE) file for the full text of the GNU General Public License v3.0.

## Contributing & Feedback
If you have suggestions, find bugs, or want to contribute to the development of the Delta Robot project, please open an **Issue** or submit a **Pull Request**.

---

## Original Credits & Authorship
This project is a high-level refactor and expansion of the department's original Delta Robot codebase. We acknowledge the foundational work provided by the NMAMIT Robotics Lab.

**Lead Architects & Developers:**
* **Likhithraj T Acharya**
* **Rajat Pandya**
* **Kshma Pai**
* **Vaibhav Kulal**

**Institution:** NMAMIT, Nitte (Department of Robotics & AI Engineering)

For detailed licensing conditions, derivative work requirements, and notification protocols, see [CONTRIBUTORS.md](CONTRIBUTORS.md).

---
