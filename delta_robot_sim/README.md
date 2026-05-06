# Delta Robot Simulation

This package handles the Gazebo Sim (Ignition) environment and the ROS 2 integration for virtual testing.

## Overview
The simulation uses `gz_ros2_control` to bridge ROS 2 controllers with the Gazebo physics engine. It provides a high-fidelity environment with realistic mass, inertia, and collision properties.

## Running the Simulation

Launch the full simulation stack (Gazebo + Spawner + Broadcasters):
```bash
ros2 launch delta_robot_sim delta_robot_spawn.launch.py
```

### Optional Arguments:
- `use_sim_feedback`: (default: true) Set to false if you are running real motors alongside sim.

## Configuration
- **`worlds/`**: Contains the `empty.sdf` world file.
- **`config/`**:
  - `ros2_controllers.yaml`: PID gains and controller update rates (1000Hz).
  - `ros_gz_bridge.yaml`: Bridges topics between ROS 2 and Gazebo transport.

## Key Components
- **`gz_ros2_control`**: The primary hardware interface for simulation.
- **Robot State Publisher**: Handles the TFs and visualization in RViz.
- **Spawn Node**: Spawns the robot at `(0, 0, 0.5)` with the correct orientation.

---
