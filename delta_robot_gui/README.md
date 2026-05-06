# Delta Robot GUI

A PyQt5-based graphical interface for real-time control and monitoring of the Delta Robot.

## Launching the GUI

### For Simulation:
```bash
ros2 launch delta_robot_gui delta_robot_gui.launch.py sim_mode:=true
```

### For Real Hardware:
```bash
ros2 launch delta_robot_gui delta_robot_gui.launch.py sim_mode:=false
```

## Features
- **Cartesian Control**: Sliders for X, Y, and Z targets.
- **Speed Tuning**: Real-time adjustment of motion speed and acceleration.
- **Task Sequencer**: Load and execute pre-defined JSON tasks or G-code.
- **Torque Management**: Enable/Disable individual motor torque (Hardware mode).
- **Feedback Visualization**: Displays current joint positions and errors.
- **Offset Controls**: Dedicated sliders for Tool and Object offsets.

## Internal Configuration
The GUI communicates via the following default topics:
- **Target Pose**: `/delta/target_pose`
- **Joint States**: `/joint_states`
- **Torque Commands**: `delta_motors/torque_command`

---
