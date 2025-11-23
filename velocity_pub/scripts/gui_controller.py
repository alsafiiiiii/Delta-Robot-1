#!/usr/bin/env python3
import sys
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose

from PyQt5.QtWidgets import (QApplication, QWidget, QVBoxLayout, QHBoxLayout, 
                             QSlider, QLabel, QGroupBox, QPushButton)
from PyQt5.QtCore import Qt, QTimer

class DeltaGUI(QWidget):
    def __init__(self):
        super().__init__()

        # --- ROS 2 Setup ---
        rclpy.init(args=None)
        self.node = rclpy.create_node('delta_gui_publisher')
        self.publisher = self.node.create_publisher(Pose, '/delta/target_pose', 10)
        
        # Timer to handle ROS spinning (keep connection alive)
        self.timer = QTimer()
        self.timer.timeout.connect(self.ros_spin)
        self.timer.start(100)

        # --- UI Constants ---
        # Scaling factors to map integer sliders to float values
        self.POS_SCALE = 1000.0   # Slider 100 -> 0.1m
        self.ROT_SCALE = 100.0    # Slider 314 -> 3.14 rad

        self.init_ui()

    def init_ui(self):
        self.setWindowTitle('Delta Robot 5-DOF Control')
        self.setGeometry(100, 100, 400, 600)

        main_layout = QVBoxLayout()

        # --- Position Group ---
        pos_group = QGroupBox("Position (Meters)")
        pos_layout = QVBoxLayout()

        # X: -0.2 to 0.2
        self.sl_x, self.lbl_x = self.create_slider("X", -200, 200, 0, pos_layout)
        # Y: -0.2 to 0.2
        self.sl_y, self.lbl_y = self.create_slider("Y", -200, 200, 0, pos_layout)
        # Z: -0.40 to -0.15 (Standard delta workspace is negative Z)
        self.sl_z, self.lbl_z = self.create_slider("Z", -400, -150, -270, pos_layout)

        pos_group.setLayout(pos_layout)
        main_layout.addWidget(pos_group)

        # --- Orientation Group ---
        rot_group = QGroupBox("Orientation (RPY Radians)")
        rot_layout = QVBoxLayout()

        # Roll (Tilt X): -pi/2 to pi/2
        self.sl_roll, self.lbl_roll = self.create_slider("Roll (Tilt)", -157, 157, 0, rot_layout)
        # Pitch (Tilt Y): -pi/2 to pi/2 (Note: Your robot might not support Y-tilt physically)
        self.sl_pitch, self.lbl_pitch = self.create_slider("Pitch (N/A)", -157, 157, 0, rot_layout)
        # Yaw (Spin Z): -pi to pi
        self.sl_yaw, self.lbl_yaw = self.create_slider("Yaw (Spin)", -314, 314, 0, rot_layout)

        rot_group.setLayout(rot_layout)
        main_layout.addWidget(rot_group)

        # --- Reset Button ---
        self.btn_reset = QPushButton("Home Position")
        self.btn_reset.clicked.connect(self.reset_sliders)
        main_layout.addWidget(self.btn_reset)

        self.setLayout(main_layout)

        # Trigger initial publish
        self.update_command()

    def create_slider(self, label_text, min_val, max_val, default_val, parent_layout):
        """Helper to create a standardized slider block"""
        container = QHBoxLayout()
        
        label = QLabel(f"{label_text}: {default_val/self.POS_SCALE:.3f}")
        label.setMinimumWidth(100)
        
        slider = QSlider(Qt.Horizontal)
        slider.setMinimum(min_val)
        slider.setMaximum(max_val)
        slider.setValue(default_val)
        slider.valueChanged.connect(lambda: self.update_label(slider, label, label_text))
        slider.valueChanged.connect(self.update_command)
        
        container.addWidget(label)
        container.addWidget(slider)
        parent_layout.addLayout(container)
        
        return slider, label

    def update_label(self, slider, label, text):
        val = slider.value()
        # Determine scale based on text context (simple heuristic)
        scale = self.ROT_SCALE if "Roll" in text or "Pitch" in text or "Yaw" in text else self.POS_SCALE
        label.setText(f"{text}: {val/scale:.3f}")

    def reset_sliders(self):
        # Block signals to prevent stuttering updates during reset
        self.blockSignals(True)
        self.sl_x.setValue(0)
        self.sl_y.setValue(0)
        self.sl_z.setValue(-270) # Nominal height
        self.sl_roll.setValue(0)
        self.sl_pitch.setValue(0)
        self.sl_yaw.setValue(0)
        self.blockSignals(False)
        
        # Manually trigger update once
        self.update_label(self.sl_x, self.lbl_x, "X")
        self.update_label(self.sl_y, self.lbl_y, "Y")
        self.update_label(self.sl_z, self.lbl_z, "Z")
        self.update_label(self.sl_roll, self.lbl_roll, "Roll (Tilt)")
        self.update_label(self.sl_pitch, self.lbl_pitch, "Pitch (N/A)")
        self.update_label(self.sl_yaw, self.lbl_yaw, "Yaw (Spin)")
        
        self.update_command()

    def get_quaternion_from_euler(self, roll, pitch, yaw):
        """
        Convert an Euler angle to a quaternion.
        Input: radians
        Output: qx, qy, qz, qw
        """
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        return [qx, qy, qz, qw]

    def update_command(self):
        # 1. Get Values
        x = self.sl_x.value() / self.POS_SCALE
        y = self.sl_y.value() / self.POS_SCALE
        z = self.sl_z.value() / self.POS_SCALE
        
        r = self.sl_roll.value() / self.ROT_SCALE
        p = self.sl_pitch.value() / self.ROT_SCALE
        y_rot = self.sl_yaw.value() / self.ROT_SCALE

        # 2. Convert RPY to Quaternion
        q = self.get_quaternion_from_euler(r, p, y_rot)

        # 3. Create Message
        msg = Pose()
        msg.position.x = x
        msg.position.y = y
        msg.position.z = z
        msg.orientation.x = q[0]
        msg.orientation.y = q[1]
        msg.orientation.z = q[2]
        msg.orientation.w = q[3]

        # 4. Publish
        self.publisher.publish(msg)

    def ros_spin(self):
        # Non-blocking spin to keep ROS 2 callbacks active (if any)
        rclpy.spin_once(self.node, timeout_sec=0)

    def closeEvent(self, event):
        self.node.destroy_node()
        rclpy.shutdown()
        event.accept()

def main():
    app = QApplication(sys.argv)
    gui = DeltaGUI()
    gui.show()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()