import threading
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from std_msgs.msg import Float32MultiArray, UInt8MultiArray
import sys
import os
import time
import collections

# Internal SDK import
try:
    from .scservo_sdk import *
except ImportError:
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from scservo_sdk import *

BAUDRATE = 1000000
DEVICE_NAME = "/dev/ttyACM0"
MOTOR_MAX_POS = 4095
RAD_TO_TICKS = 4096.0 / (2.0 * math.pi)
UP_POS = 2048.0
ALL_MOTOR_IDS = [1, 2, 3, 4, 5]


class DeltaMotorControl(Node):
    def __init__(self):
        super().__init__("delta_motor_control")
        self.get_logger().info("Delta Hardware Bridge v2.9 (Latency Corrected)")

        self.device_name = self.declare_parameter("device_name", DEVICE_NAME).value
        self.baudrate = self.declare_parameter("baudrate", BAUDRATE).value
        self.motor_joint_names = list(
            self.declare_parameter(
                "motor_joint_names",
                [
                    "motor_joint_1",
                    "motor_joint_2",
                    "motor_joint_3",
                    "motor_joint_4",
                    "motor_joint_5",
                ],
            ).value
        )

        self.hw_lock = threading.Lock()
        self.port_handler = None
        self.packet_handler = None
        self.group_sync_read = None
        self.hardware_available = False
        self.visible_motor_ids = []

        self.last_published_motor_positions = [int(UP_POS)] * 5
        self.last_commanded_radians = [0.0] * 5
        self.latest_load = [0.0] * 5
        self.latest_speed = [0.0] * 5

        self.stop_threads = False

        # Initialize hardware first
        self._initialize_hardware()

        # Publishers
        self.motor_positions_pub = self.create_publisher(
            JointState, "/delta/joint_states", 10
        )
        self.servo_actual_pub = self.create_publisher(
            Float32MultiArray, "/servo/actual", 10
        )
        self.motor_telemetry_pub = self.create_publisher(
            Float32MultiArray, "/delta/motor_telemetry", 10
        )

        # Subscribers
        self.joint_command_sub = self.create_subscription(
            JointTrajectory, "/delta/joint_commands", self.joint_command_callback, 1
        )
        self.torque_command_sub = self.create_subscription(
            UInt8MultiArray,
            "delta_motors/torque_command",
            self._torque_command_callback,
            10,
        )

        # Single feedback thread (Moves are handled in the ROS callback)
        self.feedback_thread = threading.Thread(
            target=self._feedback_worker, daemon=True
        )
        self.feedback_thread.start()

    def _initialize_hardware(self):
        with self.hw_lock:
            self.port_handler = PortHandler(self.device_name)
            self.packet_handler = sms_sts(self.port_handler)
            if not self.port_handler.openPort():
                return
            self.port_handler.setBaudRate(self.baudrate)

            # GroupSyncRead is already efficient; just need to ask for 6 bytes.
            self.group_sync_read = GroupSyncRead(
                self.packet_handler, SMS_STS_PRESENT_POSITION_L, 6
            )
            for sid in ALL_MOTOR_IDS:
                m, r, e = self.packet_handler.ping(sid)
                if r == COMM_SUCCESS:
                    self.visible_motor_ids.append(sid)
                    self.group_sync_read.addParam(sid)
                    self.packet_handler.write1ByteTxRx(sid, SMS_STS_TORQUE_ENABLE, 1)
            self.hardware_available = len(self.visible_motor_ids) > 0

    def joint_command_callback(self, msg):
        """Executes move commands. Already TxOnly due to BROADCAST_ID in SyncWrite."""
        if not self.hardware_available or not msg.points:
            return
        targets = list(msg.points[-1].positions)

        # Try lock; if we can't get it in 3ms, the feedback thread is likely holding it.
        if not self.hw_lock.acquire(timeout=0.003):
            return

        try:
            for i in range(5):
                st_id = i + 1
                if st_id not in self.visible_motor_ids:
                    continue
                theta = (
                    targets[i] if i < len(targets) else self.last_commanded_radians[i]
                )
                self.last_commanded_radians[i] = theta
                pos = int(RAD_TO_TICKS * theta + UP_POS)
                # SyncWritePosEx uses BROADCAST_ID internally -> immediate return.
                self.packet_handler.SyncWritePosEx(
                    st_id, max(0, min(MOTOR_MAX_POS, pos)), 0, 0
                )

            self.packet_handler.groupSyncWrite.txPacket()
            self.packet_handler.groupSyncWrite.clearParam()
        finally:
            self.hw_lock.release()

    def _feedback_worker(self):
        """Background loop: Polls feedback and flushes input buffer to avoid staleness."""
        while rclpy.ok() and not self.stop_threads:
            if not self.hardware_available:
                time.sleep(0.5)
                continue

            with self.hw_lock:
                # Correct flush: reset_input_buffer() on self.port_handler.ser
                if hasattr(self.port_handler, "ser"):
                    self.port_handler.ser.reset_input_buffer()

                # Now that LATENCY_TIMER is 2ms, this call won't block the system for 50ms on failure.
                self.group_sync_read.txRxPacket()
                current_pos = list(self.last_published_motor_positions)
                for i, sid in enumerate(ALL_MOTOR_IDS):
                    if sid in self.visible_motor_ids:
                        avail, _ = self.group_sync_read.isAvailable(
                            sid, SMS_STS_PRESENT_POSITION_L, 6
                        )
                        if avail:
                            p = self.group_sync_read.getData(
                                sid, SMS_STS_PRESENT_POSITION_L, 2
                            )
                            s = self.group_sync_read.getData(
                                sid, SMS_STS_PRESENT_SPEED_L, 2
                            )
                            l = self.group_sync_read.getData(
                                sid, SMS_STS_PRESENT_LOAD_L, 2
                            )
                            if p is not None:
                                current_pos[i] = self.packet_handler.scs_tohost(p, 15)
                            if s is not None:
                                self.latest_speed[i] = float(
                                    self.packet_handler.scs_tohost(s, 15)
                                )
                            if l is not None:
                                self.latest_load[i] = float(
                                    self.packet_handler.scs_tohost(l, 15)
                                )

            self._publish_feedback(current_pos)
            # Smaller sleep for higher resolution feedback now that timeouts are low.
            time.sleep(0.005)

    def _publish_feedback(self, current_pos):
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = self.motor_joint_names
        js.position = [(p - UP_POS) / RAD_TO_TICKS for p in current_pos]
        self.motor_positions_pub.publish(js)

        act_msg = Float32MultiArray()
        act_msg.data = [float(p) for p in current_pos]
        self.servo_actual_pub.publish(act_msg)

        tel_msg = Float32MultiArray()
        tel_msg.data = (
            [float(current_pos[i]) for i in range(5)]
            + self.latest_speed
            + self.latest_load
        )
        self.motor_telemetry_pub.publish(tel_msg)
        self.last_published_motor_positions = current_pos

    def _torque_command_callback(self, msg):
        if not self.hardware_available or len(msg.data) < 2:
            return
        sid, en = int(msg.data[0]), (1 if msg.data[1] else 0)
        with self.hw_lock:
            self.packet_handler.write1ByteTxRx(sid, SMS_STS_TORQUE_ENABLE, en)


def main(args=None):
    rclpy.init(args=args)
    node = DeltaMotorControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.stop_threads = True
    finally:
        if node.port_handler:
            node.port_handler.closePort()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
