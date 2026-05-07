#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory
from std_msgs.msg import Float32MultiArray
import serial
import math
import time
import re


class EspSerialBridge(Node):
    def __init__(self):
        super().__init__("esp_serial_bridge")

        # ======================================================================
        # --- 1. CONFIGURATION ---
        # ======================================================================
        self.serial_port = "/dev/ttyACM0"
        self.baud_rate = 250000

        # Robot Physical Limits
        self.MIN_DEG = 0.0
        self.MAX_DEG = 180.0

        # --- EXPANDED TO 5 TARGETS ---
        self.target_angles = [90.0, 90.0, 90.0, 90.0, 90.0]
        # ======================================================================

        # Setup Hardware
        try:
            self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=0.01)
            self.ser.reset_input_buffer()
            self.get_logger().info(
                f"Connected to {self.serial_port}. Waiting for Handshake..."
            )

            # --- HANDSHAKE LOGIC ---
            self.wait_for_handshake()

        except Exception as e:
            self.get_logger().error(f"Serial Error: {e}")
            exit(1)

        self.sensor_pattern = re.compile(r"D(\d):([\d\.]+)")

        # ROS Interface
        self.sub_traj = self.create_subscription(
            JointTrajectory, "/delta/joint_commands", self.traj_callback, 10
        )

        self.pub_sensors = self.create_publisher(
            Float32MultiArray, "/sharp_sensors/all", 10
        )
        self.get_logger().info(
            "Serial Bridge node started (100Hz Mode active for 5 Servos)"
        )

    def wait_for_handshake(self):
        while True:
            if self.ser.in_waiting > 0:
                line = self.ser.readline().decode("utf-8", errors="ignore").strip()
                if "READY" in line:
                    self.get_logger().info("✅ Handshake successful! ESP32 is ready.")
                    break
            time.sleep(0.01)

    # ==========================================================================
    # --- 2. SERIAL PROCESSING ---
    # ==========================================================================
    def process_serial_data(self):
        while self.ser.in_waiting > 0:
            try:
                line = self.ser.readline().decode("utf-8", errors="ignore").strip()

                if "D0:" in line:
                    matches = self.sensor_pattern.findall(line)
                    if (
                        len(matches) >= 3
                    ):  # Kept >= 3 so it won't crash if you don't use 5 sensors
                        matches.sort(key=lambda x: int(x[0]))
                        msg = Float32MultiArray()
                        msg.data = [float(val) for _, val in matches]
                        self.pub_sensors.publish(msg)

            except Exception as e:
                self.get_logger().warn(f"Serial Parse Error: {e}")

    def wait_and_listen(self, duration_sec):
        start_time = time.time()
        last_send_time = 0

        while (time.time() - start_time) < duration_sec:
            self.process_serial_data()

            # --- 100 HZ SENDING LOOP ---
            current_time = time.time()
            if (current_time - last_send_time) >= 0.01:
                # EXPANDED STRING FORMATTING
                cmd = f"POS:{self.target_angles[0]:.2f},{self.target_angles[1]:.2f},{self.target_angles[2]:.2f},{self.target_angles[3]:.2f},{self.target_angles[4]:.2f}\n"
                self.ser.write(cmd.encode("utf-8"))
                last_send_time = current_time

            time.sleep(0.001)

    # ==========================================================================
    # --- 3. TRAJECTORY CALLBACK ---
    # ==========================================================================
    def traj_callback(self, msg):
        if not msg.points:
            return

        for i, point in enumerate(msg.points):
            t_abs_sec = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9

            if i == 0:
                dt = t_abs_sec
            else:
                prev = msg.points[i - 1].time_from_start
                dt = t_abs_sec - (prev.sec + prev.nanosec * 1e-9)

            # Require at least 5 joint positions from ROS
            if len(point.positions) < 5:
                continue

            # Slice up to 5 positions
            raw_degs = [math.degrees(p) for p in point.positions[:5]]

            # --- CALIBRATION FIX ---
            OFFSET = 30.0
            BEVEL1_OFFSET = 90.0  # Set your desired offset for joint 4
            BEVEL2_OFFSET = 90.0  # Set your desired offset for joint 5

            corrected_degs = [
                raw_degs[0] + OFFSET,
                raw_degs[1] + OFFSET,
                raw_degs[2] + OFFSET,
                raw_degs[3] + BEVEL1_OFFSET,
                raw_degs[4] + BEVEL2_OFFSET,
            ]

            # Constrain to min/max
            final_degs = [
                max(self.MIN_DEG, min(self.MAX_DEG, d)) for d in corrected_degs
            ]

            # Update target for the 100Hz loop
            self.target_angles = final_degs

            self.wait_and_listen(dt)


def main(args=None):
    rclpy.init(args=args)
    node = EspSerialBridge()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
            node.wait_and_listen(0.01)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
