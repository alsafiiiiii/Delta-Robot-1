#!/usr/bin/env python3
import sys
import socket
import numpy as np
import threading

# ROS 2 Imports
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

# Kinematics Imports
from visual_kinematics.RobotDelta import RobotDelta
from visual_kinematics.Frame import Frame

# PyQt5 Imports
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                             QHBoxLayout, QSlider, QLabel, QPushButton, QGroupBox, QGridLayout,
                             QDoubleSpinBox)
from PyQt5.QtCore import Qt

# --- CONFIGURATION ---
ESP_IP = "10.148.4.11"   # REPLACE WITH ESP32 IP
ESP_PORT = 3333
SLIDER_SCALE = 100.0     # Factor to convert Slider Int -> Float (100 = 2 decimal places)
# ---------------------

class DeltaROSNode(Node):
    def __init__(self):
        super().__init__('delta_gui_node')
        
        # UDP Socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.esp_addr = (ESP_IP, ESP_PORT)
        
        # Robot Parameters
        self.r_base = 0.07582127019 
        self.r_ee = 0.035            
        self.l1 = 0.075              
        self.l2 = 0.2639602098       
        self.robot = RobotDelta(np.array([self.r_base, self.r_ee, self.l1, self.l2]))
        
        # ROS Publisher
        self.joint_pub = self.create_publisher(JointTrajectory, '/model/delta_robot/joint_trajectory', 10)
        self.get_logger().info(f"Connected to ESP32 at {ESP_IP}:{ESP_PORT}")

    def send_udp(self, angles_deg):
        """ Send raw degrees to ESP32 via UDP """
        # Ensure numpy array
        angles_deg = np.array(angles_deg)
        
        # Clamp
        angles_deg = np.clip(angles_deg, 0, 180)
        
        # Format: "180.00,180.00,180.00"
        packet = f"{angles_deg[0]:.2f},{angles_deg[1]:.2f},{angles_deg[2]:.2f}"
        
        try:
            self.sock.sendto(packet.encode(), self.esp_addr)
        except Exception as e:
            self.get_logger().error(f"UDP Error: {e}")

    def publish_ros_trajectory(self, angles_deg):
        """ Publish to ROS for Rviz Visualization """
        angles_deg = np.array(angles_deg)

        # Convert Servo Degrees back to Kinematics Radians for Rviz
        # Math: Servo = 90 + MathDeg -> MathDeg = Servo - 90
        math_deg = angles_deg - 90.0
        radians = np.deg2rad(math_deg)

        msg = JointTrajectory()
        msg.joint_names = ['jbf1', 'jbf2', 'jbf3']
        point = JointTrajectoryPoint()
        point.positions = [float(x) for x in radians]
        point.time_from_start = Duration(sec=0, nanosec=100000000) # 100ms
        msg.points.append(point)
        self.joint_pub.publish(msg)

    def calculate_ik(self, x, y, z):
        """ Inverse Kinematics: XYZ -> Servo Angles """
        try:
            # Frame setup
            target_frame = Frame.from_euler_3(np.array([0., 0., 0.]), np.array([[x], [y], [z]]))
            joint_angles_rad = self.robot.inverse(target_frame).flatten()
            
            # Convert to Degrees
            ik_deg = np.rad2deg(joint_angles_rad)
            
            # APPLY OFFSET: 0 math = 90 servo
            servo_deg = 180 + ik_deg
            return servo_deg
        except Exception as e:
            self.get_logger().debug(f"IK Unreachable: {e}")
            return None

    def calculate_fk(self, servo_angles_deg):
        """ Forward Kinematics: Servo Angles -> XYZ """
        try:
            # Servo (0..180) -> Math (-90..90) -> Rads
            math_deg = np.array(servo_angles_deg) - 90.0
            math_rad = np.deg2rad(math_deg)
            
            # visual_kinematics forward() returns a 3x1 array [[x], [y], [z]]
            pos = self.robot.forward(math_rad)
            
            # Flatten to [x, y, z]
            return pos.flatten() 
        except Exception as e:
            self.get_logger().debug(f"FK Error: {e}")
            return None

class MainWindow(QMainWindow):
    def __init__(self, ros_node):
        super().__init__()
        self.node = ros_node
        self.setWindowTitle("Delta Robot Commander")
        self.resize(600, 500) # Resized smaller since recording UI is gone

        # Main Layout
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        self.layout = QVBoxLayout(central_widget)

        # --- RESET & HOME BUTTONS ---
        btn_layout = QHBoxLayout()
        self.btn_reset = QPushButton("RESET (180,180,180)")
        self.btn_reset.setStyleSheet("background-color: #ffcccc; padding: 10px; font-weight: bold;")
        self.btn_reset.clicked.connect(self.action_reset)
        
        self.btn_home = QPushButton("HOME (Zero Pos)")
        self.btn_home.setStyleSheet("background-color: #ccffcc; padding: 10px;")
        self.btn_home.clicked.connect(self.action_home)

        btn_layout.addWidget(self.btn_reset)
        btn_layout.addWidget(self.btn_home)
        self.layout.addLayout(btn_layout)

        # --- XYZ CONTROLS (Inverse Kinematics) ---
        self.group_ik = QGroupBox("End Effector Position (IK)")
        ik_layout = QGridLayout()
        
        # Sliders setup (Min, Max, Default, Label)
        self.sl_x = self.create_input_row(-70.0, 70.0, 0.0, ik_layout, 0, "X (mm)")
        self.sl_y = self.create_input_row(-70.0, 70.0, 0.0, ik_layout, 1, "Y (mm)")
        self.sl_z = self.create_input_row(-250.0, -190.0, -250.0, ik_layout, 2, "Z (mm)")
        
        self.group_ik.setLayout(ik_layout)
        self.layout.addWidget(self.group_ik)

        # --- JOINT CONTROLS (Forward Kinematics) ---
        self.group_joints = QGroupBox("Servo Angles (Direct Control)")
        joint_layout = QGridLayout()
        
        self.sl_j1 = self.create_input_row(0.0, 180.0, 90.0, joint_layout, 0, "Servo 1")
        self.sl_j2 = self.create_input_row(0.0, 180.0, 90.0, joint_layout, 1, "Servo 2")
        self.sl_j3 = self.create_input_row(0.0, 180.0, 90.0, joint_layout, 2, "Servo 3")

        self.group_joints.setLayout(joint_layout)
        self.layout.addWidget(self.group_joints)
        
        # Status Label
        self.lbl_status = QLabel("Status: Ready")
        self.lbl_status.setAlignment(Qt.AlignCenter)
        self.lbl_status.setStyleSheet("font-weight: bold; font-size: 14px; color: #333;")
        self.layout.addWidget(self.lbl_status)

        # Connect signals
        self.sl_x.valueChanged.connect(self.update_from_xyz)
        self.sl_y.valueChanged.connect(self.update_from_xyz)
        self.sl_z.valueChanged.connect(self.update_from_xyz)

        self.sl_j1.valueChanged.connect(self.update_from_joints)
        self.sl_j2.valueChanged.connect(self.update_from_joints)
        self.sl_j3.valueChanged.connect(self.update_from_joints)
        
        # Flag to prevent feedback loops
        self.updating = False

    def create_input_row(self, min_val, max_val, default_val, layout, row, label_text):
        """ Creates [Label] [Slider (Int Scaled)] [DoubleSpinBox (Float)] """
        lbl_name = QLabel(label_text)
        
        # Slider (Works with Integers only)
        # We scale by SLIDER_SCALE (100) to simulate floats
        slider = QSlider(Qt.Horizontal)
        slider.setRange(int(min_val * SLIDER_SCALE), int(max_val * SLIDER_SCALE))
        slider.setValue(int(default_val * SLIDER_SCALE))
        
        # SpinBox (Works with Floats)
        spin = QDoubleSpinBox()
        spin.setRange(min_val, max_val)
        spin.setValue(default_val)
        spin.setDecimals(2)
        spin.setSingleStep(0.1)
        
        # Sync Logic: Slider -> SpinBox
        slider.valueChanged.connect(lambda val: spin.setValue(val / SLIDER_SCALE))
        
        # Sync Logic: SpinBox -> Slider
        spin.valueChanged.connect(lambda val: slider.setValue(int(val * SLIDER_SCALE)))
        
        layout.addWidget(lbl_name, row, 0)
        layout.addWidget(slider, row, 1)
        layout.addWidget(spin, row, 2)
        
        # Store reference to spinbox inside slider for easy access later
        slider.spin_ref = spin 
        return slider

    # --- ROBOT CONTROL LOGIC ---

    def action_reset(self):
        """ Move all servos to 180 and update all sliders """
        self.update_sliders_joints(180.0, 180.0, 180.0)
        self.lbl_status.setText("Status: RESET (180,180,180)")

    def action_home(self):
        """ Move to Home XYZ (0, 0, -0.25) """
        # Set values on sliders (signals will trigger update_from_xyz)
        self.sl_x.setValue(0)
        self.sl_y.setValue(0)
        self.sl_z.setValue(int(-250.0 * SLIDER_SCALE)) 
        self.lbl_status.setText("Status: HOMED")

    def update_from_xyz(self):
        """ XYZ Slider Changed -> Calculate IK -> Move Robot -> Update Joint Sliders """
        if self.updating: return
        self.updating = True

        # 1. Read XYZ Sliders (From Spinbox for float precision)
        # Input is mm, convert to meters for IK
        x = self.sl_x.spin_ref.value() / 1000.0
        y = self.sl_y.spin_ref.value() / 1000.0
        z = self.sl_z.spin_ref.value() / 1000.0
        
        # 2. Calculate IK
        angles = self.node.calculate_ik(x, y, z)
        
        if angles is not None:
            # 3. Update Joint Sliders (Visual only, no signal)
            self.sl_j1.blockSignals(True)
            self.sl_j2.blockSignals(True)
            self.sl_j3.blockSignals(True)
            self.sl_j1.spin_ref.blockSignals(True)
            self.sl_j2.spin_ref.blockSignals(True)
            self.sl_j3.spin_ref.blockSignals(True)
            
            # Set Spinbox (Float)
            self.sl_j1.spin_ref.setValue(angles[0])
            self.sl_j2.spin_ref.setValue(angles[1])
            self.sl_j3.spin_ref.setValue(angles[2])
            
            # Set Slider (Scaled Int)
            self.sl_j1.setValue(int(angles[0] * SLIDER_SCALE))
            self.sl_j2.setValue(int(angles[1] * SLIDER_SCALE))
            self.sl_j3.setValue(int(angles[2] * SLIDER_SCALE))

            self.sl_j1.blockSignals(False)
            self.sl_j2.blockSignals(False)
            self.sl_j3.blockSignals(False)
            self.sl_j1.spin_ref.blockSignals(False)
            self.sl_j2.spin_ref.blockSignals(False)
            self.sl_j3.spin_ref.blockSignals(False)

            # 4. Send Command
            self.send_command(angles)
            self.lbl_status.setText(f"IK: X{x:.3f} Y{y:.3f} Z{z:.3f}")
        else:
            self.lbl_status.setText("Status: Unreachable Position!")

        self.updating = False

    def update_from_joints(self):
        """ Joint Slider Changed -> Move Robot -> Calculate FK -> Update XYZ Sliders """
        if self.updating: return
        self.updating = True

        # 1. Read Joint Sliders (From Spinbox)
        j1 = self.sl_j1.spin_ref.value()
        j2 = self.sl_j2.spin_ref.value()
        j3 = self.sl_j3.spin_ref.value()

        # 2. Send Command
        angles = np.array([j1, j2, j3])
        self.send_command(angles)
        self.lbl_status.setText(f"Manual: {j1:.1f}, {j2:.1f}, {j3:.1f}")
        
        # 3. Update XYZ Sliders (Forward Kinematics)
        self.sync_xyz_sliders(angles)
        
        self.updating = False

    def update_sliders_joints(self, j1, j2, j3):
        """ Helper to safely update joint sliders/spins without recursion """
        self.updating = True
        
        # Block Signals
        self.sl_j1.blockSignals(True)
        self.sl_j2.blockSignals(True)
        self.sl_j3.blockSignals(True)
        self.sl_j1.spin_ref.blockSignals(True)
        self.sl_j2.spin_ref.blockSignals(True)
        self.sl_j3.spin_ref.blockSignals(True)

        # Set Values
        self.sl_j1.setValue(int(j1 * SLIDER_SCALE))
        self.sl_j2.setValue(int(j2 * SLIDER_SCALE))
        self.sl_j3.setValue(int(j3 * SLIDER_SCALE))
        self.sl_j1.spin_ref.setValue(j1)
        self.sl_j2.spin_ref.setValue(j2)
        self.sl_j3.spin_ref.setValue(j3)
        
        # Restore Signals
        self.sl_j1.blockSignals(False)
        self.sl_j2.blockSignals(False)
        self.sl_j3.blockSignals(False)
        self.sl_j1.spin_ref.blockSignals(False)
        self.sl_j2.spin_ref.blockSignals(False)
        self.sl_j3.spin_ref.blockSignals(False)
        
        # Send hardware command immediately
        self.send_command([j1, j2, j3])
        
        # Sync XYZ (FK)
        self.sync_xyz_sliders([j1, j2, j3])
        
        self.updating = False

    def sync_xyz_sliders(self, angles):
        """ Helper to set XYZ sliders from Joint Angles using FK """
        xyz = self.node.calculate_fk(angles)
        
        if xyz is not None:
            # Block Signals on XYZ
            self.sl_x.blockSignals(True)
            self.sl_y.blockSignals(True)
            self.sl_z.blockSignals(True)
            self.sl_x.spin_ref.blockSignals(True)
            self.sl_y.spin_ref.blockSignals(True)
            self.sl_z.spin_ref.blockSignals(True)
            
            x_mm = xyz[0] * 1000.0
            y_mm = xyz[1] * 1000.0
            z_mm = xyz[2] * 1000.0

            # Set Slider (Scaled)
            self.sl_x.setValue(int(x_mm * SLIDER_SCALE))
            self.sl_y.setValue(int(y_mm * SLIDER_SCALE))
            self.sl_z.setValue(int(z_mm * SLIDER_SCALE))
            
            # Set Spin (Float)
            self.sl_x.spin_ref.setValue(x_mm)
            self.sl_y.spin_ref.setValue(y_mm)
            self.sl_z.spin_ref.setValue(z_mm)

            # Restore Signals
            self.sl_x.blockSignals(False)
            self.sl_y.blockSignals(False)
            self.sl_z.blockSignals(False)
            self.sl_x.spin_ref.blockSignals(False)
            self.sl_y.spin_ref.blockSignals(False)
            self.sl_z.spin_ref.blockSignals(False)

    def send_command(self, angles):
        # Send to Hardware
        self.node.send_udp(angles)
        # Publish to ROS Visualization
        self.node.publish_ros_trajectory(angles)

def ros_spin_thread(node):
    rclpy.spin(node)

def main():
    # Init ROS
    rclpy.init(args=None)
    ros_node = DeltaROSNode()
    
    # Start ROS Spin in separate thread
    spin_thread = threading.Thread(target=ros_spin_thread, args=(ros_node,), daemon=True)
    spin_thread.start()

    # Start GUI
    app = QApplication(sys.argv)
    window = MainWindow(ros_node)
    window.show()
    
    try:
        sys.exit(app.exec_())
    finally:
        ros_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()