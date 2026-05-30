#!/usr/bin/env python3
"""
ee_tf_broadcaster.py  —  ISM330DHCX + 3x VL53L4CD → ROS2 TF

Filtering pipeline:
  raw sensor → AdaptiveKalman1D (per channel) → trig math → light EMA on XYZ

AdaptiveKalman1D auto-adjusts its gain:
  • At rest  (innovation ≈ noise floor) → gain is tiny → very smooth, no jitter
  • In motion (large innovation)        → gain → 1    → instant response, no lag

No deadband, no fixed-alpha lag. The math does the work.
"""

import math
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import serial
import re
import sys

from scipy.spatial.transform import Rotation as R


# ──────────────────────────────────────────────────────────────────────────────
# Adaptive 1-D Kalman filter
# ──────────────────────────────────────────────────────────────────────────────

class AdaptiveKalman1D:
    """
    Scalar Kalman (constant-position model) with innovation-scaled process noise.

    Q_adaptive = Q_base + Q_scale * innovation²

    At rest  (innovation ≈ σ_sensor): Q stays near Q_base → K tiny  → smooth.
    In motion (innovation >> σ_sensor): Q grows with i²   → K → 1   → instant.

    Parameters
    ----------
    R        : measurement noise variance (σ_sensor²)
    Q_base   : minimum process noise — sets the smoothing floor
    Q_scale  : how fast Q grows with innovation — sets responsiveness
    """

    __slots__ = ("R", "Q_base", "Q_scale", "_x", "_P")

    def __init__(self, R: float, Q_base: float, Q_scale: float):
        self.R       = R
        self.Q_base  = Q_base
        self.Q_scale = Q_scale
        self._x: float | None = None
        self._P: float        = R

    def update(self, z: float) -> float:
        if self._x is None:
            self._x = z
            self._P = self.Q_base
            return z

        innovation = z - self._x
        Q          = self.Q_base + self.Q_scale * (innovation * innovation)
        P_pred     = self._P + Q
        K          = P_pred / (P_pred + self.R)

        self._x   += K * innovation
        self._P    = (1.0 - K) * P_pred
        return self._x

    def seed(self, v: float) -> None:
        self._x = v
        self._P = self.Q_base

    @property
    def value(self) -> float:
        return self._x if self._x is not None else 0.0


class SimpleEMA:
    __slots__ = ("alpha", "_v")

    def __init__(self, alpha: float):
        self.alpha = alpha
        self._v: float | None = None

    def update(self, z: float) -> float:
        if self._v is None:
            self._v = z
        else:
            self._v += self.alpha * (z - self._v)
        return self._v

    @property
    def value(self) -> float:
        return self._v if self._v is not None else 0.0


# ──────────────────────────────────────────────────────────────────────────────
# ROS2 node
# ──────────────────────────────────────────────────────────────────────────────

class EETfBroadcaster(Node):

    # ── Tuning constants ──────────────────────────────────────────────────────
    # TOF: noise σ ≈ 2 mm  →  R = σ² = 4
    TOF_R       = 4.0
    TOF_Q_BASE  = 0.10   # lower = smoother at rest
    TOF_Q_SCALE = 0.05   # higher = faster response to motion

    # IMU angles: noise σ ≈ 0.03°  →  R = σ² ≈ 0.001
    ANGLE_R       = 0.001
    ANGLE_Q_BASE  = 0.0001
    ANGLE_Q_SCALE = 0.002

    # Final XYZ: light EMA post-trig (Kalman did the heavy lifting)
    POS_ALPHA = 0.50

    K_VCSEL = 0.045
    HOME_N  = 20
    # ─────────────────────────────────────────────────────────────────────────

    def __init__(self):
        super().__init__("ee_tf_broadcaster")
        self.tf_broadcaster = TransformBroadcaster(self)

        self.port_name = self.declare_parameter("port_name", "/dev/ttyUSB0").value
        self.baudrate = int(self.declare_parameter("baudrate", 115200).value)
        self.update_rate_hz = float(self.declare_parameter("update_rate_hz", 200.0).value)

        try:
            self.serial_port = serial.Serial(self.port_name, self.baudrate, timeout=0.01)
            self.get_logger().info(f"Connected to {self.port_name} at {self.baudrate} baud.")
        except Exception as e:
            self.get_logger().error(f"Failed to open serial port: {e}")
            sys.exit(1)

        _tof = dict(R=self.TOF_R,   Q_base=self.TOF_Q_BASE,   Q_scale=self.TOF_Q_SCALE)
        _ang = dict(R=self.ANGLE_R, Q_base=self.ANGLE_Q_BASE, Q_scale=self.ANGLE_Q_SCALE)

        self.kf_t1    = AdaptiveKalman1D(**_tof)
        self.kf_t2    = AdaptiveKalman1D(**_tof)
        self.kf_t3    = AdaptiveKalman1D(**_tof)
        self.kf_roll  = AdaptiveKalman1D(**_ang)
        self.kf_pitch = AdaptiveKalman1D(**_ang)
        self.kf_yaw   = AdaptiveKalman1D(**_ang)

        self.ema_x = SimpleEMA(self.POS_ALPHA)
        self.ema_y = SimpleEMA(self.POS_ALPHA)
        self.ema_z = SimpleEMA(self.POS_ALPHA)

        self.is_homed     = False
        self.home_samples = []
        self.HOME_T1 = self.HOME_T2 = self.HOME_T3 = 0.0

        self.regex = re.compile(
            r"R:\s*([-+]?\d*\.?\d+)\s*P:\s*([-+]?\d*\.?\d+)\s*Y:\s*([-+]?\d*\.?\d+)"
            r"\s*\|\s*dX:\s*([-+]?\d*\.?\d+)\s*\|\s*dY:\s*([-+]?\d*\.?\d+)"
            r"\s*\|\s*dZ:\s*([-+]?\d*\.?\d+)"
        )

        self.update_rate_hz = max(1.0, self.update_rate_hz)
        self.timer = self.create_timer(1.0 / self.update_rate_hz, self.timer_callback)
        self.get_logger().info("Ready - hold at home position for auto-homing...")

    def timer_callback(self):
        try:
            while self.serial_port.in_waiting > 0:
                line = self.serial_port.readline().decode("utf-8", errors="ignore").strip()
                if not line:
                    continue

                tokens = [t.strip() for t in line.split(",")]
                parsed = False

                if len(tokens) >= 9:
                    try:
                        self.process_data(
                            float(tokens[0]), float(tokens[1]), float(tokens[2]),
                            float(tokens[6]), float(tokens[7]), float(tokens[8]),
                            is_dt=True,
                        )
                        parsed = True
                    except ValueError:
                        pass

                if not parsed:
                    m = self.regex.search(line)
                    if m:
                        self.process_data(
                            float(m.group(1)), float(m.group(2)), float(m.group(3)),
                            float(m.group(4)), float(m.group(5)), float(m.group(6)),
                            is_dt=False,
                        )
        except Exception:
            pass

    def process_data(self,
                     raw_roll: float, raw_pitch: float, raw_yaw: float,
                     v1: float, v2: float, v3: float,
                     is_dt: bool = False):

        # 1. Filter angles in sensor frame, then remap axes
        roll  = self.kf_roll.update(raw_pitch)
        pitch = self.kf_pitch.update(raw_roll)
        yaw   = self.kf_yaw.update(-raw_yaw)
        rot_true = R.from_euler("xyz", [roll, pitch, yaw], degrees=True)

        if is_dt:
            # 2. Homing
            if not self.is_homed:
                self.home_samples.append((v1, v2, v3))
                if len(self.home_samples) >= self.HOME_N:
                    n = self.HOME_N
                    self.HOME_T1 = sum(s[0] for s in self.home_samples) / n
                    self.HOME_T2 = sum(s[1] for s in self.home_samples) / n
                    self.HOME_T3 = sum(s[2] for s in self.home_samples) / n
                    self.kf_t1.seed(self.HOME_T1)
                    self.kf_t2.seed(self.HOME_T2)
                    self.kf_t3.seed(self.HOME_T3)
                    self.is_homed = True
                    self.get_logger().info(
                        f"[OK] AUTO-HOMED - "
                        f"T1={self.HOME_T1:.1f}  T2={self.HOME_T2:.1f}  T3={self.HOME_T3:.1f} mm"
                    )
                return

            # 3. Filter raw TOF (pre-trig — critical)
            t1_f = self.kf_t1.update(v1)
            t2_f = self.kf_t2.update(v2)
            t3_f = self.kf_t3.update(v3)

            # 4. Tilt + VCSEL cone compensation
            rot_mat = rot_true.as_matrix()
            cos_x = abs(rot_mat[0, 0])
            cos_y = abs(rot_mat[1, 1])
            cos_z = abs(rot_mat[2, 2])

            angle_x = math.acos(min(cos_x, 1.0))
            angle_y = math.acos(min(cos_y, 1.0))
            angle_z = math.acos(min(cos_z, 1.0))

            t1_comp = t1_f * (1.0 + self.K_VCSEL * angle_x ** 2)
            t2_comp = t2_f * (1.0 + self.K_VCSEL * angle_y ** 2)
            t3_comp = t3_f * (1.0 + self.K_VCSEL * angle_z ** 2)

            delta_x =  t1_comp * cos_x - self.HOME_T1
            delta_y =  t2_comp * cos_y - self.HOME_T2
            delta_z =  t3_comp * cos_z - self.HOME_T3

            x_m =  delta_x / 1000.0
            y_m = -delta_y / 1000.0
            z_m = (delta_z - 375.0) / 1000.0

        else:
            x_m =  v2 / 1000.0
            y_m = -v1 / 1000.0
            z_m = (v3 - 375.0) / 1000.0

        # 5. Light final EMA
        x_out = self.ema_x.update(x_m)
        y_out = self.ema_y.update(y_m)
        z_out = self.ema_z.update(z_m)

        # 6. Publish
        t = TransformStamped()
        t.header.stamp    = self.get_clock().now().to_msg()
        t.header.frame_id = "delta_robot/world_link"
        t.child_frame_id  = "ee_link"
        t.transform.translation.x = x_out
        t.transform.translation.y = y_out
        t.transform.translation.z = z_out

        q = rot_true.as_quat()
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = EETfBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if hasattr(node, "serial_port") and node.serial_port.is_open:
            node.serial_port.close()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()