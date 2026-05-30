# Delta Hardware Bridge

This package provides the low-level communication bridge between ROS 2 and the physical Delta Robot motors.

## Hardware Details
The robot uses **Serial Bus Servos** which communicate via half-duplex UART data packets.

### Supported Motors:
- **ID 1, 2, 3**: **Feetech ST3215-HS** (High-speed bicep servos)
- **ID 4, 5**: **Feetech STS3032** (Compact differential servos for the end-effector)

### Communication Specs:
- **Baud Rate**: 921,600 baud
- **Port**: `/dev/ttyUSB0` (default)
- **Protocol**: Custom Serial/Binary bridge (uses SET/SETN commands)

## Running the Bridge

Before running, ensure you have permissions for the serial port:
```bash
sudo chmod 666 /dev/ttyUSB0
```

Start the bridge node:
```bash
ros2 run delta_hardware_bridge motor_control_node
```

## Features
- **Automatic Discovery**: Scans and pings IDs 1-5 on startup.
- **Feedback Streaming**: Reads real-time position and velocity from servos at 50Hz.
- **Torque Control**: Supports enabling/disabling torque via the `delta_motors/torque_command` topic.
- **Binary Mode**: Supports high-efficiency binary packet transmission to reduce latency.

## Published Topics
- `/delta/joint_states`: Real-time feedback from the motors (radians).
- `/servo/actual`: Raw tick positions for plotting/debugging.

## Subscribed Topics
- `/delta/joint_commands`: Target joint angles (radians) to be sent to the motors.
- `delta_motors/torque_command`: [ID, State] pairs for torque management.

---
