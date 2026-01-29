#!/usr/bin/env python3
import sys
import math
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Pose, Twist
from std_msgs.msg import Bool, Float64

from PyQt5.QtWidgets import (QApplication, QWidget, QVBoxLayout, QHBoxLayout, 
                             QSlider, QLabel, QGroupBox, QPushButton, QCheckBox)
from PyQt5.QtCore import Qt, QTimer, QThread, pyqtSignal

# --- WORKER THREAD FOR ROS 2 ---
class RosThread(QThread):
    def __init__(self, node):
        super().__init__()
        self.node = node

    def run(self):
        # This blocks until the node is destroyed, but since it's 
        # in a separate thread, the GUI remains responsive.
        rclpy.spin(self.node) 

    def stop(self):
        self.quit()
        self.wait()

class DeltaGUI(QWidget):
    def __init__(self):
        super().__init__()

        # --- ROS 2 Setup ---
        # Initialize ROS if it hasn't been initialized yet
        if not rclpy.ok():
            rclpy.init(args=None)
            
        self.node = rclpy.create_node('delta_gui_publisher')
        self.publisher = self.node.create_publisher(Pose, '/delta/target_pose', 10)
        self.speed_publisher = self.node.create_publisher(Twist, '/delta/speed_params', 10)
        self.suction_pub = self.node.create_publisher(Bool, '/suction/command', 10)
        self.conveyor_pub = self.node.create_publisher(Float64, '/conveyor/cmd_vel', 10)
        
        # --- THREADING FIX ---
        # Instead of a QTimer calling spin_once(), we run the spin loop
        # in a dedicated separate thread.
        self.ros_thread = RosThread(self.node)
        self.ros_thread.start()

        # --- UI Constants ---
        self.POS_SCALE = 1000.0   # Slider 100 -> 0.1m
        self.ROT_SCALE = 100.0    # Slider 314 -> 3.14 rad
        self.SPEED_SCALE = 1000.0 # Slider 100 -> 0.1 m/s
        self.ANG_SPEED_SCALE = 10.0 # Slider 10 -> 1.0 rad/s
        
        # Keyboard control
        self.keyboard_enabled = False
        self.KEYBOARD_STEP = 5 
        self.keys_pressed = set()
        
        # Keyboard update timer (Keep this one! It handles UI logic, not ROS)
        self.kb_timer = QTimer()
        self.kb_timer.timeout.connect(self.keyboard_update)
        self.kb_timer.start(50)  
        
        self.init_ui()

    def init_ui(self):
        self.setWindowTitle('Delta Robot 5-DOF Control')
        self.setGeometry(100, 100, 400, 700)

        main_layout = QVBoxLayout()

        # --- Keyboard Control Toggle ---
        kb_group = QGroupBox("Keyboard Control")
        kb_layout = QVBoxLayout()
        
        self.kb_checkbox = QCheckBox("Enable WASD + Space/Ctrl Navigation")
        self.kb_checkbox.stateChanged.connect(self.toggle_keyboard_mode)
        kb_layout.addWidget(self.kb_checkbox)
        
        kb_help = QLabel("WASD: XY | Space/X: Up/Down | Q/E: Yaw | Z/C: Roll | H: Home")
        kb_help.setStyleSheet("color: gray; font-size: 10px;")
        kb_layout.addWidget(kb_help)
        
        # Step size slider
        step_container = QHBoxLayout()
        self.lbl_step = QLabel(f"Step Size: {self.KEYBOARD_STEP} mm")
        self.sl_step = QSlider(Qt.Horizontal)
        self.sl_step.setMinimum(1)
        self.sl_step.setMaximum(20)
        self.sl_step.setValue(5)
        self.sl_step.valueChanged.connect(self.update_step_size)
        step_container.addWidget(self.lbl_step)
        step_container.addWidget(self.sl_step)
        kb_layout.addLayout(step_container)
        
        kb_group.setLayout(kb_layout)
        main_layout.addWidget(kb_group)

        # --- Position Group ---
        pos_group = QGroupBox("Position (Meters)")
        pos_layout = QVBoxLayout()

        self.sl_x, self.lbl_x = self.create_slider("X", -200, 200, 0, pos_layout)
        self.sl_y, self.lbl_y = self.create_slider("Y", -200, 200, 0, pos_layout)
        self.sl_z, self.lbl_z = self.create_slider("Z", -400, -150, -270, pos_layout)

        pos_group.setLayout(pos_layout)
        main_layout.addWidget(pos_group)

        # --- Orientation Group ---
        rot_group = QGroupBox("End-Effector Orientation")
        rot_layout = QVBoxLayout()

        self.sl_roll, self.lbl_roll = self.create_slider("Roll (Tilt)", -157, 157, 0, rot_layout)
        self.sl_yaw, self.lbl_yaw = self.create_slider("Yaw (Spin)", -314, 314, 0, rot_layout)
        
        rot_group.setLayout(rot_layout)
        main_layout.addWidget(rot_group)

        # --- Speed Group ---
        speed_group = QGroupBox("Speed Settings")
        speed_layout = QVBoxLayout()

        self.sl_lin_speed, self.lbl_lin_speed = self.create_slider("Linear Speed (m/s)", 500, 1000, 300, speed_layout)
        self.sl_ang_speed, self.lbl_ang_speed = self.create_slider("Angular Speed (rad/s)", 1, 50, 10, speed_layout)

        speed_group.setLayout(speed_layout)
        main_layout.addWidget(speed_group)

        self.btn_reset = QPushButton("Home Position (H)")
        self.btn_reset.clicked.connect(self.reset_sliders)
        main_layout.addWidget(self.btn_reset)
        
        # --- Conveyor & Suction Controls ---
        ctrl_group = QGroupBox("Conveyor & Suction")
        ctrl_layout = QHBoxLayout()
        
        self.btn_conv_start = QPushButton("▶ Start Belt")
        self.btn_conv_start.clicked.connect(lambda: self.set_conveyor(-0.05))
        ctrl_layout.addWidget(self.btn_conv_start)
        
        self.btn_conv_stop = QPushButton("⏹ Stop Belt")
        self.btn_conv_stop.clicked.connect(lambda: self.set_conveyor(0.0))
        ctrl_layout.addWidget(self.btn_conv_stop)
        
        self.btn_suction_on = QPushButton("🔴 Suction ON")
        self.btn_suction_on.clicked.connect(lambda: self.set_suction(True))
        ctrl_layout.addWidget(self.btn_suction_on)
        
        self.btn_suction_off = QPushButton("⚪ Suction OFF")
        self.btn_suction_off.clicked.connect(lambda: self.set_suction(False))
        ctrl_layout.addWidget(self.btn_suction_off)
        
        ctrl_group.setLayout(ctrl_layout)
        main_layout.addWidget(ctrl_group)

        self.setLayout(main_layout)
        self.setFocusPolicy(Qt.StrongFocus)
        self.update_command()

    def toggle_keyboard_mode(self, state):
        self.keyboard_enabled = (state == Qt.Checked)
        if self.keyboard_enabled:
            self.setFocus()
            self.node.get_logger().info("Keyboard control ENABLED")
        else:
            self.node.get_logger().info("Keyboard control DISABLED")
    
    def update_step_size(self, value):
        self.KEYBOARD_STEP = value
        self.lbl_step.setText(f"Step Size: {value} mm")

    def keyPressEvent(self, event):
        if not self.keyboard_enabled:
            super().keyPressEvent(event)
            return
        if event.isAutoRepeat():
            return
        self.keys_pressed.add(event.key())
    
    def keyReleaseEvent(self, event):
        if event.isAutoRepeat():
            return
        self.keys_pressed.discard(event.key())
    
    def keyboard_update(self):
        if not self.keyboard_enabled or not self.keys_pressed:
            return
        
        step = self.KEYBOARD_STEP
        rot_step = 5 
        
        # Position
        if Qt.Key_W in self.keys_pressed: self.sl_y.setValue(min(self.sl_y.maximum(), self.sl_y.value() + step))
        if Qt.Key_S in self.keys_pressed: self.sl_y.setValue(max(self.sl_y.minimum(), self.sl_y.value() - step))
        if Qt.Key_A in self.keys_pressed: self.sl_x.setValue(max(self.sl_x.minimum(), self.sl_x.value() - step))
        if Qt.Key_D in self.keys_pressed: self.sl_x.setValue(min(self.sl_x.maximum(), self.sl_x.value() + step))
        if Qt.Key_Space in self.keys_pressed: self.sl_z.setValue(min(self.sl_z.maximum(), self.sl_z.value() + step))
        if Qt.Key_X in self.keys_pressed: self.sl_z.setValue(max(self.sl_z.minimum(), self.sl_z.value() - step))
        
        # Orientation
        if Qt.Key_Q in self.keys_pressed: self.sl_yaw.setValue(max(self.sl_yaw.minimum(), self.sl_yaw.value() - rot_step))
        if Qt.Key_E in self.keys_pressed: self.sl_yaw.setValue(min(self.sl_yaw.maximum(), self.sl_yaw.value() + rot_step))
        if Qt.Key_Z in self.keys_pressed: self.sl_roll.setValue(max(self.sl_roll.minimum(), self.sl_roll.value() - rot_step))
        if Qt.Key_C in self.keys_pressed: self.sl_roll.setValue(min(self.sl_roll.maximum(), self.sl_roll.value() + rot_step))
        
        if Qt.Key_H in self.keys_pressed:
            self.reset_sliders()
            self.keys_pressed.discard(Qt.Key_H)

    def create_slider(self, label_text, min_val, max_val, default_val, parent_layout, scale=None):
        container = QHBoxLayout()
        if scale is None: scale = self.POS_SCALE
        
        label = QLabel(f"{label_text}: {default_val/scale:.3f}")
        label.setMinimumWidth(120)
        
        slider = QSlider(Qt.Horizontal)
        slider.setMinimum(min_val)
        slider.setMaximum(max_val)
        slider.setValue(default_val)
        slider.setProperty("scale", scale)
        slider.valueChanged.connect(lambda: self.update_label(slider, label, label_text))
        slider.valueChanged.connect(self.update_command)
        
        container.addWidget(label)
        container.addWidget(slider)
        parent_layout.addLayout(container)
        return slider, label

    def update_label(self, slider, label, text):
        val = slider.value()
        scale = slider.property("scale")
        if scale is None: scale = self.POS_SCALE
        label.setText(f"{text}: {val/scale:.3f}")

    def reset_sliders(self):
        self.blockSignals(True)
        self.sl_x.setValue(0)
        self.sl_y.setValue(0)
        self.sl_z.setValue(-270)
        self.sl_roll.setValue(0)
        self.sl_yaw.setValue(0)
        self.sl_lin_speed.setValue(50)
        self.sl_ang_speed.setValue(10)
        self.blockSignals(False)
        self.update_label(self.sl_x, self.lbl_x, "X")
        self.update_label(self.sl_y, self.lbl_y, "Y")
        self.update_label(self.sl_z, self.lbl_z, "Z")
        self.update_label(self.sl_roll, self.lbl_roll, "Roll (Tilt)")
        self.update_label(self.sl_yaw, self.lbl_yaw, "Yaw (Spin)")
        self.update_label(self.sl_lin_speed, self.lbl_lin_speed, "Linear Speed (m/s)")
        self.update_label(self.sl_ang_speed, self.lbl_ang_speed, "Angular Speed (rad/s)")
        self.update_command()

    def get_quaternion_from_euler(self, roll, pitch, yaw):
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        return [qx, qy, qz, qw]

    def update_command(self):
        # Publishing from the GUI thread is safe in rclpy for simple publishers
        x = self.sl_x.value() / self.POS_SCALE
        y = self.sl_y.value() / self.POS_SCALE
        z = self.sl_z.value() / self.POS_SCALE
        
        r = self.sl_roll.value() / self.ROT_SCALE
        p = 0.0
        y_rot = self.sl_yaw.value() / self.ROT_SCALE

        q = self.get_quaternion_from_euler(r, p, y_rot)

        msg = Pose()
        msg.position.x = x
        msg.position.y = y
        msg.position.z = z
        msg.orientation.x = q[0]
        msg.orientation.y = q[1]
        msg.orientation.z = q[2]
        msg.orientation.w = q[3]

        self.publisher.publish(msg)

        speed_msg = Twist()
        speed_msg.linear.x = self.sl_lin_speed.value() / self.SPEED_SCALE
        speed_msg.angular.z = self.sl_ang_speed.value() / self.ANG_SPEED_SCALE
        self.speed_publisher.publish(speed_msg)
    
    def set_conveyor(self, velocity):
        msg = Float64()
        msg.data = velocity
        self.conveyor_pub.publish(msg)
        self.node.get_logger().info(f"Conveyor: {velocity}")
    
    def set_suction(self, state):
        msg = Bool()
        msg.data = state
        self.suction_pub.publish(msg)
        self.node.get_logger().info(f"Suction: {'ON' if state else 'OFF'}")

    def closeEvent(self, event):
        # Clean shutdown of thread and node
        self.kb_timer.stop()
        
        if hasattr(self, 'ros_thread'):
            # Stop the ROS thread
            if self.ros_thread.isRunning():
                # We can't easily stop rclpy.spin() from outside, 
                # but destroying the node usually triggers it to return
                self.node.destroy_node()
                self.ros_thread.quit()
                self.ros_thread.wait()
        
        if rclpy.ok():
            rclpy.shutdown()
        event.accept()

def main():
    # Helper to prevent shared library issues on some Linux distros
    import os
    os.environ["QT_QPA_PLATFORM"] = "xcb" 
    
    app = QApplication(sys.argv)
    gui = DeltaGUI()
    gui.show()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()