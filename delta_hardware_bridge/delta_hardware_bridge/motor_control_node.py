#!/usr/bin/env python3

import glob
import math
import errno
import time
import rclpy
from rclpy.node import Node
import serial

from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from std_msgs.msg import Float32MultiArray, UInt8MultiArray, String

# ST Servo Constants
BAUDRATE = 921600
DEVICE_NAME = "/dev/ttyUSB0"
MAX_ST_MOVING_SPEED = 0
MAX_ST_MOVING_ACC = 0
SERIAL_TIMEOUT_S = 0.08
SERIAL_CMD_WAIT_S = 0.12

BIN_SOF = 0x7E
BIN_SETN = 0x05

# Converting from radians to motor position mapping
# Motor Position 0-4095 corresponds to 360 degrees (2*pi radians)
# 2048 is the center position (0 radians)
MOTOR_MAX_POS = 4095
RAD_TO_TICKS = 4096.0 / (2.0 * math.pi)
UP_POS = 2048.0

# Velocity limit range is 0~7500. ST3215-HS uses 50 steps/sec ≈ 0.732 RPM.
VEL_UNIT_RPM = 0.732 / 50.0  # units in RPM per (step/sec)
RAD_S_TO_REV_MIN = 60.0 / (2.0 * math.pi)

BICEP_IDS = [1, 2, 3]
EE_IDS = [4, 5]
ALL_MOTOR_IDS = BICEP_IDS + EE_IDS

DEFAULT_STREAM_PERIOD_MS = 20
DEFAULT_JOINT_NAMES = [
    "motor_joint_1",
    "motor_joint_2",
    "motor_joint_3",
    "motor_joint_4",
    "motor_joint_5",
]


class DeltaMotorControl(Node):
    def __init__(self):
        super().__init__("delta_motor_control")
        self.get_logger().info("Python DeltaMotorControl Node Started")

        self.qos_depth = self.declare_parameter("qos_depth", 10).value
        self.device_name = self.declare_parameter("device_name", DEVICE_NAME).value
        self.baudrate = int(self.declare_parameter("baudrate", BAUDRATE).value)
        self.bicep_moving_speed = int(self.declare_parameter("moving_speed", 0).value)
        self.bicep_moving_acc = int(self.declare_parameter("moving_acc", 0).value)
        self.ee_moving_speed = int(self.declare_parameter("ee_moving_speed", 0).value)
        self.ee_moving_acc = int(self.declare_parameter("ee_moving_acc", 0).value)
        self.max_write_retries = int(self.declare_parameter("max_write_retries", 3).value)
        self.read_fail_watchdog_limit = int(self.declare_parameter("read_fail_watchdog_limit", 5).value)
        self.enable_velocity_feedback = bool(
            self.declare_parameter("enable_velocity_feedback", False).value
        )
        self.serial_timeout_s = float(
            self.declare_parameter("serial_timeout_s", SERIAL_TIMEOUT_S).value
        )
        self.serial_cmd_wait_s = float(
            self.declare_parameter("serial_cmd_wait_s", SERIAL_CMD_WAIT_S).value
        )
        self.use_binary_bridge = bool(
            self.declare_parameter("use_binary_bridge", True).value
        )
        self.binary_noack_set = bool(
            self.declare_parameter("binary_noack_set", True).value
        )
        self.stream_feedback_period_ms = int(
            self.declare_parameter("stream_feedback_period_ms", DEFAULT_STREAM_PERIOD_MS).value
        )
        self.motor1_center_ticks = int(
            self.declare_parameter("motor1_center_ticks", int(UP_POS)).value
        )
        self.motor2_center_ticks = int(
            self.declare_parameter("motor2_center_ticks", int(UP_POS)).value
        )
        self.motor3_center_ticks = int(
            self.declare_parameter("motor3_center_ticks", int(UP_POS)).value
        )
        self.motor4_center_ticks = int(
            self.declare_parameter("motor4_center_ticks", int(UP_POS)).value
        )
        self.motor5_center_ticks = int(
            self.declare_parameter("motor5_center_ticks", int(UP_POS)).value
        )
        self.joint_command_topic = self.declare_parameter(
            "joint_command_topic", "/delta/joint_commands"
        ).value
        self.joint_state_topic = self.declare_parameter(
            "joint_state_topic", "/delta/joint_states"
        ).value
        self.motor_joint_names = list(
            self.declare_parameter("motor_joint_names", DEFAULT_JOINT_NAMES).value
        )
        self.motor_ids = list(
            self.declare_parameter("motor_ids", ALL_MOTOR_IDS).value
        )

        self.bicep_moving_speed = max(0, min(MAX_ST_MOVING_SPEED, self.bicep_moving_speed))
        self.bicep_moving_acc = max(0, min(MAX_ST_MOVING_ACC, self.bicep_moving_acc))
        self.ee_moving_speed = max(0, min(MAX_ST_MOVING_SPEED, self.ee_moving_speed))
        self.ee_moving_acc = max(0, min(MAX_ST_MOVING_ACC, self.ee_moving_acc))
        self.max_write_retries = max(1, self.max_write_retries)
        self.read_fail_watchdog_limit = max(1, self.read_fail_watchdog_limit)
        self.stream_feedback_period_ms = max(10, self.stream_feedback_period_ms)
        self.consecutive_read_failures = 0

        self.center_ticks_by_id = {
            1: max(0, min(MOTOR_MAX_POS, self.motor1_center_ticks)),
            2: max(0, min(MOTOR_MAX_POS, self.motor2_center_ticks)),
            3: max(0, min(MOTOR_MAX_POS, self.motor3_center_ticks)),
            4: max(0, min(MOTOR_MAX_POS, self.motor4_center_ticks)),
            5: max(0, min(MOTOR_MAX_POS, self.motor5_center_ticks)),
        }

        # Serial-bridge I/O state.
        self.serial_port = None
        self.hardware_available = False
        self.visible_motor_ids = []
        self.missing_motor_ids = list(ALL_MOTOR_IDS)
        self.bridge_binary_enabled = False
        self.bin_seq = 1
        # Feedback state: only populated when real feedback arrives
        self.latest_motor_positions = {}
        self.latest_motor_velocities = {}
        # Track last published positions to retain state when feedback is missing.
        # Use None to indicate we have no prior reading yet.
        self.last_published_motor_positions = [None] * len(ALL_MOTOR_IDS)
        self.last_commanded_radians = [0.0] * len(ALL_MOTOR_IDS)
        # Have we received at least one valid position sample for the visible motors?
        self._first_valid_feedback_received = False
        # Set of motor IDs for which we've received valid feedback
        self._valid_feedback_ids = set()

        # Publisher to notify UI/apps about initialization status (mode failures, etc.)
        self.init_status_pub = self.create_publisher(String, "delta_motors/init_status", 10)

        self._initialize_hardware()

        # Subscribers
        self.joint_command_sub = self.create_subscription(
            JointTrajectory,
            self.joint_command_topic,
            self.joint_command_callback,
            self.qos_depth,
        )

        # Publishers
        self.motor_positions_pub = self.create_publisher(
            JointState, self.joint_state_topic, 10
        )
        self.servo_target_pub = self.create_publisher(
            Float32MultiArray, "/servo/target", 10
        )
        self.servo_actual_pub = self.create_publisher(
            Float32MultiArray, "/servo/actual", 10
        )

        # Subscriber for torque control commands: [motor_id, enable] (e.g., [1, 0] = motor 1 OFF)
        self.torque_command_sub = self.create_subscription(
            UInt8MultiArray,
            "delta_motors/torque_command",
            self._torque_command_callback,
            10,
        )

        # Timer rate is driven by the configured feedback stream period.
        stream_period_s = max(0.001, self.stream_feedback_period_ms / 1000.0)
        self.timer = self.create_timer(stream_period_s, self.timer_callback)

    def _get_candidate_ports(self, preferred_port):
        """Return likely serial ports with preferred first when present."""
        detected_ports = sorted(glob.glob("/dev/ttyUSB*")) + sorted(glob.glob("/dev/ttyACM*"))

        candidates = []
        if preferred_port:
            candidates.append(preferred_port)

        for port in detected_ports:
            if port not in candidates:
                candidates.append(port)

        return candidates

    def _read_serial_lines(self, window_s=None, stop_prefixes=()):
        if not self.serial_port:
            return []
        if window_s is None:
            window_s = self.serial_cmd_wait_s
        end_t = self.get_clock().now().nanoseconds / 1e9 + window_s
        out = []
        while (self.get_clock().now().nanoseconds / 1e9) < end_t:
            raw = self.serial_port.readline()
            if not raw:
                continue
            line = raw.decode("utf-8", errors="replace").strip()
            if not line:
                continue
            out.append(line)
            if stop_prefixes and any(line.startswith(p) for p in stop_prefixes):
                break
        return out

    def _bridge_request(self, cmd, wait_s=None, stop_prefixes=()):
        if not self.serial_port:
            return []
        self.serial_port.write((cmd.strip() + "\n").encode("ascii", errors="ignore"))
        return self._read_serial_lines(wait_s, stop_prefixes)

    def _bin_crc_xor(self, cmd, seq, payload):
        crc = 0
        crc ^= (cmd & 0xFF)
        crc ^= (seq & 0xFF)
        crc ^= (len(payload) & 0xFF)
        for b in payload:
            crc ^= b
        return crc & 0xFF

    def _build_bin_frame(self, cmd, seq, payload):
        crc = self._bin_crc_xor(cmd, seq, payload)
        return bytes([BIN_SOF, cmd & 0xFF, seq & 0xFF, len(payload) & 0xFF]) + payload + bytes([crc])

    def _parse_feedback_line(self, line):
        if not line:
            return False
        if not (line.startswith("FBP id=") or line.startswith("FBPS id=")):
            return False

        parts = line.split()
        sid = None
        pos = None
        spd = None
        for p in parts:
            if p.startswith("id="):
                raw = p.split("=", 1)[1]
                if raw.isdigit():
                    sid = int(raw)
            elif p.startswith("pos="):
                raw = p.split("=", 1)[1]
                if raw.lstrip("-").isdigit():
                    pos = int(raw)
            elif p.startswith("speed="):
                raw = p.split("=", 1)[1]
                if raw.lstrip("-").isdigit():
                    spd = int(raw)

        if sid is None or sid not in ALL_MOTOR_IDS or pos is None:
            return False

        # Save feedback (accept pos==0 as a valid reading)
        self.latest_motor_positions[sid] = pos
        if spd is not None:
            self.latest_motor_velocities[sid] = spd
        # Mark this servo as having valid feedback
        self._valid_feedback_ids.add(sid)
        self._first_valid_feedback_received = True
        return True

    def _drain_serial_feedback(self, max_lines=200):
        if not self.serial_port:
            return 0

        updates = 0
        drained = 0
        while drained < max_lines:
            try:
                waiting = self.serial_port.in_waiting
            except (OSError, serial.SerialException) as exc:
                self._handle_serial_io_error("checking in_waiting", exc)
                return -1
            if waiting <= 0:
                break
            try:
                raw = self.serial_port.readline()
            except (OSError, serial.SerialException) as exc:
                self._handle_serial_io_error("reading serial feedback", exc)
                return -1
            if not raw:
                break
            line = raw.decode("utf-8", errors="replace").strip()
            drained += 1
            if self._parse_feedback_line(line):
                updates += 1

        return updates

    def _handle_serial_io_error(self, operation, exc):
        err_no = getattr(exc, "errno", None)
        err_name = errno.errorcode.get(err_no, "UNKNOWN") if isinstance(err_no, int) else "UNKNOWN"
        self.get_logger().error(
            f"Serial I/O error while {operation} on {self.device_name}: {exc} "
            f"(errno={err_no}, code={err_name}). Disabling hardware I/O."
        )
        try:
            if self.serial_port is not None:
                self.serial_port.close()
        except Exception as close_exc:  # noqa: BLE001
            self.get_logger().warning(f"Failed to close serial port after I/O error: {close_exc}")
        self.serial_port = None
        self.hardware_available = False

    def _send_setn_binary(self, entries):
        if not self.serial_port or not entries:
            return False
        payload = bytearray()
        for sid, pos, speed, acc in entries:
            payload.append(sid & 0xFF)
            payload.extend((pos & 0xFF, (pos >> 8) & 0xFF))
            payload.extend((speed & 0xFF, (speed >> 8) & 0xFF))
            payload.append(acc & 0xFF)

        frame = self._build_bin_frame(BIN_SETN, self.bin_seq, payload)
        self.serial_port.write(frame)
        self.bin_seq = (self.bin_seq + 1) & 0xFF
        return True

    def _parse_list_ids(self, lines):
        for ln in lines:
            if ln.startswith("OK ids="):
                payload = ln.split("=", 1)[1].strip()
                if not payload:
                    return []
                ids = []
                for tok in payload.split(","):
                    tok = tok.strip()
                    if tok.isdigit():
                        ids.append(int(tok))
                return ids
        return []

    def _ping_servo(self, servo_id):
        lines = self._bridge_request(
            f"PING {servo_id}",
            wait_s=0.12,
            stop_prefixes=("OK ping", "ERR"),
        )
        return any(ln.startswith("OK ping") for ln in lines)

    def _initialize_hardware(self):
        candidate_ports = self._get_candidate_ports(self.device_name)
        if not candidate_ports:
            self.get_logger().warning(
                "No serial ports found (expected /dev/ttyUSB* or /dev/ttyACM*); running in no-hardware mode"
            )
            self.hardware_available = False
            return

        self.get_logger().info(f"Trying serial ports: {candidate_ports}")
        port_errors = []

        for port_path in candidate_ports:
            try:
                trial = serial.Serial(
                    port=port_path,
                    baudrate=self.baudrate,
                    timeout=self.serial_timeout_s,
                )
                self.serial_port = trial
                self.device_name = port_path
                self.get_logger().info(f"Connected on {port_path} @ {self.baudrate}")
                # Let bridge settle after open/reset.
                self._bridge_request("HELP", wait_s=0.25)
                break
            except Exception as exc:
                port_errors.append(f"{port_path}: {exc}")

        if self.serial_port is None:
            self.get_logger().warning("Could not open any serial port; running in no-hardware mode")
            for err in port_errors:
                self.get_logger().warning(f"  - {err}")
            self.hardware_available = False
            return

        self._bridge_request("STREAM 0", wait_s=0.15, stop_prefixes=("OK stream", "ERR"))
        self._bridge_request("SCANVERBOSE 0", wait_s=0.15, stop_prefixes=("OK scan_verbose", "ERR"))
        if self.use_binary_bridge:
            bin_rsp = self._bridge_request("BIN 1", wait_s=0.15, stop_prefixes=("OK bin=", "ERR"))
            self.bridge_binary_enabled = any(ln.startswith("OK bin=1") for ln in bin_rsp)
            if self.bridge_binary_enabled:
                bfast_rsp = self._bridge_request(
                    f"BFAST {1 if self.binary_noack_set else 0}",
                    wait_s=0.15,
                    stop_prefixes=("OK bfast=", "ERR"),
                )
                if not any(ln.startswith("OK bfast=") for ln in bfast_rsp):
                    self.get_logger().warning("BFAST not acknowledged; continuing with binary mode")
        else:
            self.bridge_binary_enabled = False

        self._bridge_request("SCAN", wait_s=0.25, stop_prefixes=("OK scan_started", "ERR"))
        time.sleep(1.0)
        ids = self._parse_list_ids(
            self._bridge_request("LIST", wait_s=0.35, stop_prefixes=("OK ids=",))
        )
        if not ids:
            # Fallback to direct ping probing for expected IDs.
            ids = [sid for sid in ALL_MOTOR_IDS if self._ping_servo(sid)]

        self.visible_motor_ids = sorted([sid for sid in ids if sid in ALL_MOTOR_IDS])
        self.missing_motor_ids = [sid for sid in ALL_MOTOR_IDS if sid not in self.visible_motor_ids]

        self.get_logger().info(
            f"Visible motors ({len(self.visible_motor_ids)}/{len(ALL_MOTOR_IDS)}): {self.visible_motor_ids}"
        )
        if self.missing_motor_ids:
            self.get_logger().warning(f"Missing motors: {self.missing_motor_ids}")

        if not self.visible_motor_ids:
            self.get_logger().warning("No servos detected; running in no-hardware mode")
            self.serial_port.close()
            self.serial_port = None
            self.hardware_available = False
            return

        failed_mode_ids = []
        for st_id in self.visible_motor_ids:
            # Enable torque first
            self._bridge_request(
                f"TORQUE {st_id} 1", wait_s=0.15, stop_prefixes=("OK torque", "ERR")
            )
            self.get_logger().info(f"Torque enabled for Motor ID: {st_id}")

            # Request servo mode (Mode 0)
            mode_rsp = self._bridge_request(
                f"MODE {st_id} 0", wait_s=0.15, stop_prefixes=("OK mode", "ERR")
            )
            if any(ln.startswith(f"OK mode id={st_id}") for ln in mode_rsp):
                self.get_logger().info(f"Servo mode set for Motor ID: {st_id}")
            else:
                self.get_logger().warning(f"Motor ID {st_id} servo mode not confirmed by MODE response")

            # Verify servo mode by reading actual mode with AREAD
            verify_rsp = self._bridge_request(
                f"AREAD {st_id}", wait_s=0.15, stop_prefixes=("FBA", "ERR")
            )
            mode_verified = False
            for line in verify_rsp:
                if f"id={st_id}" in line and "mode=0" in line:
                    mode_verified = True
                    self.get_logger().info(f"Motor ID {st_id} verified in servo mode")
                    break

            if not mode_verified:
                self.get_logger().warning(f"Motor ID {st_id} servo mode verification failed")
                failed_mode_ids.append(st_id)
            else:
                # Only move to midpoint when servo mode is verified
                self._bridge_request(
                    f"MIDDLE {st_id}", wait_s=0.15, stop_prefixes=("OK middle", "ERR")
                )
                self.get_logger().info(f"Motor ID {st_id} moved to midpoint")

        # If any motors failed mode verification, publish a status message for GUI/user
        if failed_mode_ids:
            try:
                msg = String()
                msg.data = f"MODE_VERIFY_FAIL ids={failed_mode_ids}"
                self.init_status_pub.publish(msg)
            except Exception:
                # Non-fatal; log and continue
                self.get_logger().warning(f"Failed to publish init status for failed IDs: {failed_mode_ids}")

        self._bridge_request("TMODE POS", wait_s=0.15, stop_prefixes=("OK tmode", "ERR"))
        self._bridge_request(
            f"STREAM 1 {self.stream_feedback_period_ms}",
            wait_s=0.2,
            stop_prefixes=("OK stream", "ERR"),
        )

        self.hardware_available = True

    def convert_to_radians(self, motor_pos, st_id):
        center = self.center_ticks_by_id.get(st_id, int(UP_POS))
        return (motor_pos - center) / RAD_TO_TICKS

    def convert_to_motor_position(self, theta, st_id):
        center = self.center_ticks_by_id.get(st_id, int(UP_POS))
        motor_pos = RAD_TO_TICKS * theta + center
        return int(max(0, min(MOTOR_MAX_POS, motor_pos)))

    def convert_to_motor_velocity(self, theta_vel):
        rpm = RAD_S_TO_REV_MIN * theta_vel
        # Convert rpm to step/sec
        raw = int(abs(rpm / VEL_UNIT_RPM))
        return max(0, min(MAX_ST_MOVING_SPEED, raw))

    def speed_acc_for_id(self, st_id):
        if st_id in EE_IDS:
            return self.ee_moving_speed, self.ee_moving_acc
        return self.bicep_moving_speed, self.bicep_moving_acc

    def _extract_target_positions(self, msg):
        if not msg.points:
            return None
        point = msg.points[-1]
        if not point.positions:
            return None

        positions = list(point.positions)
        names = list(msg.joint_names)
        if names and self.motor_joint_names:
            name_to_index = {name: idx for idx, name in enumerate(names)}
            targets = []
            for motor_name in self.motor_joint_names:
                idx = name_to_index.get(motor_name)
                if idx is None or idx >= len(positions):
                    targets.append(None)
                else:
                    targets.append(positions[idx])
            if any(val is not None for val in targets):
                return targets

        if len(positions) >= len(ALL_MOTOR_IDS):
            return positions[: len(ALL_MOTOR_IDS)]
        return None

    def joint_command_callback(self, msg):
        if not self.hardware_available:
            self.get_logger().debug("Ignoring joint command: hardware unavailable")
            return

        targets = self._extract_target_positions(msg)
        if targets is None:
            self.get_logger().debug("Joint command missing usable positions")
            return

        motor_positions = []
        for idx in range(len(ALL_MOTOR_IDS)):
            if idx < len(targets) and targets[idx] is not None:
                self.last_commanded_radians[idx] = float(targets[idx])
            theta = self.last_commanded_radians[idx]
            motor_positions.append(self.convert_to_motor_position(theta, idx + 1))

        all_ok = True
        entries = []
        for i, pos in enumerate(motor_positions):
            st_id = i + 1
            if st_id not in self.visible_motor_ids:
                continue
            st_moving_speed, st_moving_acc = self.speed_acc_for_id(st_id)
            entries.append((st_id, pos, st_moving_speed, st_moving_acc))

        if self.bridge_binary_enabled and entries:
            all_ok = self._send_setn_binary(entries)
            if not all_ok:
                self.get_logger().error("SETN binary send failed; falling back to text SET")

        if (not self.bridge_binary_enabled) or (not all_ok):
            all_ok = True
            for st_id, pos, st_moving_speed, st_moving_acc in entries:
                lines = self._bridge_request(
                    f"SET {st_id} {pos} {st_moving_speed} {st_moving_acc}",
                    wait_s=0.15,
                    stop_prefixes=("OK set", "ERR"),
                )
                if not any(ln.startswith("OK set") for ln in lines):
                    all_ok = False
                    self.get_logger().error(
                        f"[ID:{st_id:03d}] SET failed via serial bridge"
                    )

        if all_ok:
            # self.get_logger().info(f"SyncWrite sent: {motor_positions}") # Removed for performance at 100Hz
            # Publish to /servo/target for plotter
            target_msg = Float32MultiArray()
            target_msg.data = [float(p) for p in motor_positions]
            self.servo_target_pub.publish(target_msg)
        self.get_logger().debug(f"Motor Positions Set: {motor_positions} [motor ticks]")


    def timer_callback(self):
        if not self.hardware_available or self.serial_port is None:
            return

        updates = self._drain_serial_feedback(max_lines=300)
        if updates < 0:
            return
        if updates == 0:
            self.consecutive_read_failures += 1
        else:
            self.consecutive_read_failures = 0

        if self.consecutive_read_failures >= self.read_fail_watchdog_limit:
            self.get_logger().warning(
                "Read watchdog tripped after %d failures; disabling hardware I/O"
                % self.consecutive_read_failures
            )
            self.hardware_available = False
            return

        # If we haven't yet received valid feedback for all visible motors, skip publishing.
        if self.visible_motor_ids and not self._valid_feedback_ids.issuperset(self.visible_motor_ids):
            missing = sorted(set(self.visible_motor_ids) - self._valid_feedback_ids)
            self.get_logger().debug(f"Awaiting feedback for motors {missing}; skipping publish")
            return

        # Initialize with last published positions to retain state when feedback is missing
        motor_positions = list(self.last_published_motor_positions)
        motor_velocities = [0] * len(ALL_MOTOR_IDS)
        for st_id in self.visible_motor_ids:
            pos = self.latest_motor_positions.get(st_id, None)
            spd = self.latest_motor_velocities.get(st_id, 0)

            # Validate bounds; accept 0 as a valid reading
            if pos is None:
                continue
            if pos < 0 or pos > MOTOR_MAX_POS:
                continue

            motor_positions[st_id - 1] = pos
            motor_velocities[st_id - 1] = spd

        # Ensure all positions are ints for conversion; fall back to center for unknown slots
        for i in range(len(motor_positions)):
            if motor_positions[i] is None:
                motor_positions[i] = int(UP_POS)

        # Convert and publish
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = list(self.motor_joint_names)

        position_out = []
        velocity_out = []
        for idx, _name in enumerate(self.motor_joint_names):
            if idx >= len(motor_positions):
                break
            position_out.append(self.convert_to_radians(motor_positions[idx], idx + 1))
            if self.enable_velocity_feedback:
                velocity_out.append(
                    motor_velocities[idx] * VEL_UNIT_RPM / RAD_S_TO_REV_MIN
                )

        joint_state.position = position_out
        if self.enable_velocity_feedback:
            joint_state.velocity = velocity_out

        self.motor_positions_pub.publish(joint_state)

        # Update last published positions to retain state when feedback temporarily missing
        self.last_published_motor_positions = list(motor_positions)

        # Publish to /servo/actual for plotter
        actual_msg = Float32MultiArray()
        actual_msg.data = [float(p) for p in motor_positions]
        self.servo_actual_pub.publish(actual_msg)

    def _torque_command_callback(self, msg):
        """Handle torque control commands: [motor_id, enable] pairs."""
        if not self.hardware_available or not self.serial_port:
            self.get_logger().warning("Torque command received but hardware not available")
            return
        
        try:
            # Parse command: expects [motor_id, enable] where enable is 0 or 1
            if len(msg.data) >= 2:
                motor_id = int(msg.data[0])
                enable = int(msg.data[1])
                
                # Clamp values
                if self.motor_ids:
                    motor_id = max(min(self.motor_ids), min(max(self.motor_ids), motor_id))
                else:
                    motor_id = max(1, min(5, motor_id))
                enable = 1 if enable else 0
                
                # Send TORQUE command to ESP32 board
                cmd = f"TORQUE {motor_id} {enable}"
                response = self._bridge_request(cmd, wait_s=0.15, stop_prefixes=("OK torque", "ERR"))
                
                if any("OK torque" in line for line in response):
                    self.get_logger().info(f"Torque command sent: Motor {motor_id} -> {'ON' if enable else 'OFF'}")
                else:
                    self.get_logger().warning(f"Torque command failed for motor {motor_id}: {response}")
        except Exception as e:
            self.get_logger().error(f"Error processing torque command: {str(e)}")


def main(args=None):
    rclpy.init(args=args)
    node = DeltaMotorControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if hasattr(node, "serial_port") and node.serial_port is not None:
            node.serial_port.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
