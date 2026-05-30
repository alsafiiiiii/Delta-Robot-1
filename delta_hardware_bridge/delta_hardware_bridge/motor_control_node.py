import threading
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from std_msgs.msg import Float32MultiArray, UInt8MultiArray
import time
import serial          # pyserial

# ── Arduino serial settings ───────────────────────────────────────────────────
ARDUINO_PORT      = "/dev/ttyUSB0"    # Arduino port (CH340 clone)
ARDUINO_BAUDRATE  = 115200            # CRITICAL: Must match Arduino Serial.begin()
ARDUINO_TIMEOUT   = 0.05              # seconds — non-blocking readline

# ── NEMA34 conversion ─────────────────────────────────────────────────────────
# Must match STEPS_PER_REV in the Arduino firmware.
STEPS_PER_REV   = 4000
RAD_TO_STEPS    = STEPS_PER_REV / (2.0 * math.pi)

# ── Default motion params sent to Arduino on startup ─────────────────────────
NEMA34_MAX_SPEED    = 30000          # steps/s
NEMA34_ACCEL        = 10000         # steps/s²

# ── Motor Directions (1 for normal, -1 for reverse) ──────────────────────────
# Change these values to flip the rotation direction of specific motors in software
MOTOR_DIRS          = [1, 1, 1]   


class ArduinoBridge:
    """
    Manages the serial link to the Arduino Uno.

    Outgoing commands are queued and sent by a dedicated writer thread so
    the ROS callback never blocks on serial I/O.

    Incoming STATUS lines are parsed and stored in `positions` / `alarms`.
    """

    def __init__(self, port: str, baudrate: int, logger):
        self.logger     = logger
        self.positions  = [0, 0, 0]       # step counts from Arduino
        self.alarms     = [False, False, False]
        self._lock      = threading.Lock()
        self._cmd_queue: list[str] = []
        self._stop      = False

        try:
            self._ser = serial.Serial(port, baudrate, timeout=ARDUINO_TIMEOUT)
            # CRITICAL: Arduino Uno requires ~3 seconds to exit its bootloader after connection
            time.sleep(3.5)            
            self._ser.reset_input_buffer()
            self.connected = True
            logger.info(f"Arduino connected on {port} @ {baudrate} baud")
        except serial.SerialException as e:
            self.connected = False
            logger.error(f"Arduino not found on {port}: {e}")
            return

        # Reader thread: parses STATUS / ACK / ERR
        self._reader = threading.Thread(target=self._read_loop, daemon=True)
        self._reader.start()

        # Writer thread: drains command queue
        self._writer = threading.Thread(target=self._write_loop, daemon=True)
        self._writer.start()

        # Send initial motion parameters
        for mid in [1, 2, 3]:
            self.send(f"SPEED {mid} {NEMA34_MAX_SPEED}")
            self.send(f"ACCEL {mid} {NEMA34_ACCEL}")

    # ── Public API ────────────────────────────────────────────────────────────

    def send(self, cmd: str):
        """Queue a command for sending (non-blocking)."""
        if not self.connected:
            return
        with self._lock:
            self._cmd_queue.append(cmd + "\n")

    def move_to_radians(self, motor_id: int, radians: float):
        # Multiply the target radians by the direction array value (-1 or 1)
        direction = MOTOR_DIRS[motor_id - 1]
        steps = int(radians * RAD_TO_STEPS * direction)
        self.send(f"MOVE {motor_id} {steps}")

    def set_enable(self, motor_id: int, enable: bool):
        self.send(f"ENABLE {motor_id} {1 if enable else 0}")

    def get_position_radians(self, motor_id: int) -> float:
        # Also flip the incoming position so ROS2 sees the correct coordinates
        direction = MOTOR_DIRS[motor_id - 1]
        return (self.positions[motor_id - 1] / RAD_TO_STEPS) * direction

    def get_alarm(self, motor_id: int) -> bool:
        return self.alarms[motor_id - 1]

    def close(self):
        self._stop = True
        if self.connected:
            self._ser.close()

    # ── Internal threads ──────────────────────────────────────────────────────

    def _write_loop(self):
        while not self._stop:
            with self._lock:
                if self._cmd_queue:
                    cmd = self._cmd_queue.pop(0)
                else:
                    cmd = None
            if cmd:
                try:
                    self._ser.write(cmd.encode())
                except serial.SerialException as e:
                    self.logger.error(f"Arduino write error: {e}")
            else:
                time.sleep(0.001)   # idle back-off

    def _read_loop(self):
        while not self._stop:
            try:
                raw = self._ser.readline()
            except (serial.SerialException, TypeError):
                if not self._stop:
                    self.logger.error("Arduino read error (port closed or disconnected)")
                break
            except Exception as e:
                if not self._stop:
                    self.logger.error(f"Arduino read error: {e}")
                break

            if not raw:
                continue

            line = raw.decode(errors="ignore").strip()
            if not line:
                continue

            if line.startswith("STATUS"):
                self._parse_status(line)
            elif line.startswith("ERR"):
                self.logger.warn(f"Arduino: {line}")

    def _parse_status(self, line: str):
        # "STATUS <pos1> <pos2> <pos3> <alm1> <alm2> <alm3>"
        parts = line.split()
        if len(parts) != 7:
            return
        try:
            self.positions = [int(parts[1]), int(parts[2]), int(parts[3])]
            self.alarms    = [bool(int(parts[4])),
                              bool(int(parts[5])),
                              bool(int(parts[6]))]
        except ValueError:
            pass


# ── Main ROS2 node ────────────────────────────────────────────────────────────

class DeltaMotorControl(Node):
    def __init__(self):
        super().__init__("delta_motor_control")
        self.get_logger().info("Delta Hardware Bridge v5.1 (Motors 1-3: NEMA34 via Arduino Uno | Direction Control Active)")

        # ── ROS parameters ────────────────────────────────────────────────────
        self.arduino_port = self.declare_parameter("arduino_port", ARDUINO_PORT).value
        self.motor_joint_names = list(
            self.declare_parameter(
                "motor_joint_names",
                [
                    "motor_joint_1",
                    "motor_joint_2",
                    "motor_joint_3",
                ],
            ).value
        )

        # ── State ─────────────────────────────────────────────────────────────
        self.last_commanded_radians = [0.0] * 3
        self.stop_threads = False

        # ── Arduino bridge (motors 1, 2, 3) ───────────────────────────────────
        self.arduino = ArduinoBridge(
            self.arduino_port, ARDUINO_BAUDRATE, self.get_logger()
        )

        # ── Publishers ────────────────────────────────────────────────────────
        self.motor_positions_pub = self.create_publisher(
            JointState,        "/delta/joint_states",    10
        )
        self.servo_actual_pub = self.create_publisher(
            Float32MultiArray, "/servo/actual",          10
        )
        self.motor_telemetry_pub = self.create_publisher(
            Float32MultiArray, "/delta/motor_telemetry",  10
        )

        # ── Subscribers ───────────────────────────────────────────────────────
        self.joint_command_sub = self.create_subscription(
            JointTrajectory,
            "/delta/joint_commands",
            self.joint_command_callback,
            1,
        )
        self.torque_command_sub = self.create_subscription(
            UInt8MultiArray,
            "delta_motors/torque_command",
            self._torque_command_callback,
            10,
        )

        # ── Alarm monitor timer (100 ms) ──────────────────────────────────────
        self.create_timer(0.1, self._alarm_monitor)

        # ── Feedback thread ───────────────────────────────────────────────────
        self.feedback_thread = threading.Thread(
            target=self._feedback_worker, daemon=True
        )
        self.feedback_thread.start()


    # ── Joint command callback ────────────────────────────────────────────────

    def joint_command_callback(self, msg: JointTrajectory):
        if not msg.points:
            return
        targets = list(msg.points[-1].positions)

        # ── Motors 1, 2, 3 → Arduino / NEMA34 ─────────────────────────────────
        for motor_id in [1, 2, 3]:
            i = motor_id - 1
            theta = targets[i] if i < len(targets) else self.last_commanded_radians[i]
            self.last_commanded_radians[i] = theta

            if self.arduino.get_alarm(motor_id):
                self.get_logger().warn(
                    f"Motor {motor_id} alarm active — command suppressed. "
                    "Clear fault on HBS860H driver, then power-cycle ENA."
                )
                continue

            self.arduino.move_to_radians(motor_id, theta)

    # ── Alarm monitor ─────────────────────────────────────────────────────────

    def _alarm_monitor(self):
        if not self.arduino.connected:
            return
        for motor_id in [1, 2, 3]:
            if self.arduino.get_alarm(motor_id):
                self.get_logger().error(
                    f"[ALARM] Motor {motor_id} (NEMA34/HBS860H) fault! "
                    "Check driver LED — power-cycle or press reset on driver."
                )

    # ── Feedback worker ───────────────────────────────────────────────────────

    def _feedback_worker(self):
        while rclpy.ok() and not self.stop_threads:
            # NEMA34 positions: live from Arduino STATUS packets
            nema_pos_rad = [
                self.arduino.get_position_radians(mid) for mid in [1, 2, 3]
            ]

            self._publish_feedback(nema_pos_rad)
            time.sleep(0.005)   # ~200 Hz

    def _publish_feedback(self, nema_pos_rad: list):
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name     = self.motor_joint_names
        js.position = nema_pos_rad
        self.motor_positions_pub.publish(js)

        # /servo/actual: NEMA34 steps (raw)
        nema_steps_raw = [float(self.arduino.positions[i]) for i in range(3)]
        act_msg        = Float32MultiArray()
        act_msg.data   = nema_steps_raw
        self.servo_actual_pub.publish(act_msg)

        # Sending positions with dummy values (0.0) for speed and load to maintain 9-element telemetry format
        tel_msg      = Float32MultiArray()
        tel_msg.data = [float(p) for p in nema_pos_rad] + [0.0, 0.0, 0.0] + [0.0, 0.0, 0.0]
        self.motor_telemetry_pub.publish(tel_msg)

    # ── Torque / enable command ────────────────────────────────────────────────

    def _torque_command_callback(self, msg: UInt8MultiArray):
        if len(msg.data) < 2:
            return
        sid    = int(msg.data[0])
        enable = bool(msg.data[1])

        if 1 <= sid <= 3:
            self.arduino.set_enable(sid, enable)
            self.get_logger().info(
                f"Motor {sid} (NEMA34) {'enabled' if enable else 'disabled'} via Arduino."
            )

    # ── Cleanup ───────────────────────────────────────────────────────────────

    def _cleanup(self):
        self.stop_threads = True
        self.arduino.close()


# ── Entry point ───────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = DeltaMotorControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._cleanup()
        node.destroy_node()
        # Guard against ROS2 calling shutdown internally on KeyboardInterrupt
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()