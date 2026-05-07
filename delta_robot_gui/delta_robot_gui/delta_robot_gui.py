#!/usr/bin/env python3
"""Versatile delta robot GUI for Cartesian moves, G-code, and JSON tasks."""

import os
import sys
import math
import time
from dataclasses import dataclass

import rclpy
from PyQt5.QtCore import QEvent, QProcess, QPoint, Qt, QTimer
from PyQt5.QtGui import QGuiApplication
from PyQt5.QtWidgets import (
    QApplication,
    QCheckBox,
    QComboBox,
    QDialog,
    QDoubleSpinBox,
    QFileDialog,
    QFrame,
    QFormLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QGridLayout,
    QSlider,
    QSpacerItem,
    QSizePolicy,
    QTabWidget,
    QPlainTextEdit,
    QVBoxLayout,
    QWidget,
    QGraphicsDropShadowEffect,
    QTextEdit,
    QSpinBox,
    QListWidget,
    QListWidgetItem,
    QRadioButton,
)
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Pose, Twist
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import UInt8MultiArray, String, Float64MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


@dataclass
class SliderSpec:
    label: str
    minimum_mm: int
    maximum_mm: int
    default_mm: int


DEFAULT_JOINT_NAMES = [
    "motor_joint_1",
    "motor_joint_2",
    "motor_joint_3",
    "motor_joint_4",
    "motor_joint_5",
]
RAD_TO_TICKS = 4096.0 / (2.0 * math.pi)
UP_POS = 2048.0


class MotorFeedbackNode(Node):
    def __init__(self, joint_names, joint_state_topic):
        super().__init__("delta_motor_feedback_window")
        self.joint_names = list(joint_names)
        self.latest_angles = {name: None for name in self.joint_names}
        self.create_subscription(
            JointState,
            joint_state_topic,
            self._feedback_callback,
            10,
        )

    def _feedback_callback(self, msg):
        name_to_index = {name: idx for idx, name in enumerate(msg.name)}
        for name in self.joint_names:
            idx = name_to_index.get(name)
            if idx is None or idx >= len(msg.position):
                continue
            self.latest_angles[name] = msg.position[idx]


class MotorAnglesWindow(QDialog):
    def __init__(self, joint_names, joint_state_topic, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Motor Angles")
        self.setMinimumSize(620, 300)

        self.joint_names = list(joint_names)
        self.node = MotorFeedbackNode(self.joint_names, joint_state_topic)
        self.angle_labels = {}

        layout = QVBoxLayout(self)
        layout.setContentsMargins(14, 14, 14, 14)
        layout.setSpacing(10)

        title = QLabel("Live motor_position_feedback")
        title.setObjectName("titleLabel")
        title.setStyleSheet("font-size: 18px; padding: 0;")
        layout.addWidget(title)

        subtitle = QLabel("Shows actual joint angles for configured joints.")
        subtitle.setObjectName("hintLabel")
        subtitle.setStyleSheet("padding: 0; background: transparent;")
        layout.addWidget(subtitle)

        grid = QGridLayout()
        grid.setHorizontalSpacing(12)
        grid.setVerticalSpacing(8)
        grid.addWidget(QLabel("Motor"), 0, 0)
        grid.addWidget(QLabel("Actual angle"), 0, 1)

        for row, joint_name in enumerate(self.joint_names, start=1):
            motor_label = QLabel(joint_name)
            motor_label.setStyleSheet("font-weight: 700;")
            angle_label = QLabel("Waiting for feedback...")
            angle_label.setObjectName("previewLabel")
            angle_label.setStyleSheet(
                "color: #d7ecff; padding: 6px 10px; background: rgba(42, 108, 176, 0.24); border: 1px solid rgba(96, 169, 238, 0.45); border-radius: 8px;"
            )
            grid.addWidget(motor_label, row, 0)
            grid.addWidget(angle_label, row, 1)
            self.angle_labels[joint_name] = angle_label

        layout.addLayout(grid)

        self.status_label = QLabel(f"Listening to {joint_state_topic}...")
        self.status_label.setObjectName("feedbackLabel")
        layout.addWidget(self.status_label)

        self.poll_timer = QTimer(self)
        self.poll_timer.timeout.connect(self._poll_feedback)
        self.poll_timer.start(50)

        self._poll_feedback()

    def _format_angle(self, value: float) -> str:
        return f"{value:+.4f} rad   ({math.degrees(value):+.1f} deg)"

    def _poll_feedback(self):
        if rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.0)

        latest_angles = dict(self.node.latest_angles)
        received_any = False
        for joint_name in self.joint_names:
            value = latest_angles.get(joint_name)
            if value is None:
                continue
            received_any = True
            self.angle_labels[joint_name].setText(self._format_angle(value))

        if received_any:
            self.status_label.setText("Receiving live motor feedback.")

    def closeEvent(self, event):
        self.poll_timer.stop()
        self.node.destroy_node()
        event.accept()


class DeltaGuiNode(Node):
    def __init__(self):
        super().__init__("delta_robot_gui")
        self.declare_parameter("sim_mode", True)
        self.declare_parameter("target_pose_topic", "/delta/target_pose")
        self.declare_parameter("speed_topic", "/delta/speed_params")
        self.declare_parameter(
            "joint_command_topic",
            "/joint_trajectory_controller/joint_trajectory",
        )
        self.declare_parameter("joint_state_topic", "/joint_states")
        self.declare_parameter("hardware_joint_command_topic", "/delta/joint_commands")
        self.declare_parameter("hardware_joint_state_topic", "/delta/joint_states")
        self.declare_parameter("auto_hardware_detect", True)
        self.declare_parameter("torque_command_topic", "delta_motors/torque_command")
        self.declare_parameter("init_status_topic", "delta_motors/init_status")
        self.declare_parameter("joint_names", DEFAULT_JOINT_NAMES)
        self.declare_parameter("motor_ids", [1, 2, 3, 4, 5])

        self.sim_mode = bool(self.get_parameter("sim_mode").value)
        self.target_pose_topic = self.get_parameter("target_pose_topic").value
        self.speed_topic = self.get_parameter("speed_topic").value
        self.joint_command_topic = self.get_parameter("joint_command_topic").value
        self.joint_state_topic = self.get_parameter("joint_state_topic").value
        self.hardware_joint_command_topic = self.get_parameter(
            "hardware_joint_command_topic"
        ).value
        self.hardware_joint_state_topic = self.get_parameter(
            "hardware_joint_state_topic"
        ).value
        self.auto_hardware_detect = bool(self.get_parameter("auto_hardware_detect").value)
        self.torque_command_topic = self.get_parameter("torque_command_topic").value
        self.init_status_topic = self.get_parameter("init_status_topic").value
        self.joint_names = list(self.get_parameter("joint_names").value)
        self.motor_ids = list(self.get_parameter("motor_ids").value)

        self.pose_publisher = self.create_publisher(Pose, self.target_pose_topic, 10)
        self.speed_publisher = self.create_publisher(Twist, self.speed_topic, 10)
        self.offset_publisher = self.create_publisher(Float64MultiArray, "/delta/offsets", 10)
        self.joint_command_publisher = self.create_publisher(
            JointTrajectory, self.joint_command_topic, 10
        )
        self.hardware_joint_command_publisher = self.create_publisher(
            JointTrajectory, self.hardware_joint_command_topic, 10
        )
        self.torque_command_publisher = self.create_publisher(
            UInt8MultiArray, self.torque_command_topic, 10
        )
        self.hardware_feedback_subscription = self.create_subscription(
            JointState,
            self.hardware_joint_state_topic,
            self._hardware_joint_state_callback,
            10,
        )
        self.joint_state_subscription = self.create_subscription(
            JointState,
            self.joint_state_topic,
            self._joint_state_callback,
            10,
        )

        self.hardware_feedback_time = None
        self.latest_feedback_time = None

    def _hardware_joint_state_callback(self, msg):
        self._handle_joint_state(msg, is_hardware=True)

    def _joint_state_callback(self, msg):
        self._handle_joint_state(msg, is_hardware=False)

    def _handle_joint_state(self, msg, is_hardware: bool):
        self.latest_feedback_time = time.time()
        if is_hardware:
            self.hardware_feedback_time = self.latest_feedback_time


    def hardware_available(self) -> bool:
        if not self.auto_hardware_detect:
            return False
        if self.hardware_feedback_time is None:
            return False
        return (time.time() - self.hardware_feedback_time) < 1.0

    def send_torque_command(self, motor_id: int, enable: int):
        if self.sim_mode and not self.hardware_available():
            return f"TORQUE {motor_id} {enable} (sim no-op)"
        try:
            msg = UInt8MultiArray()
            msg.data = [int(motor_id), int(enable)]
            self.torque_command_publisher.publish(msg)
            return f"TORQUE {motor_id} {enable}"
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"Failed to send torque command: {str(exc)}")
            return None

    def publish_pose(self, x_m: float, y_m: float, z_m: float, tilt_rad: float, spin_rad: float):
        msg = Pose()
        msg.position.x = float(x_m)
        msg.position.y = float(y_m)
        msg.position.z = float(z_m)
        qx, qy, qz, qw = self._euler_to_quaternion(tilt_rad, 0.0, spin_rad)
        msg.orientation.x = qx
        msg.orientation.y = qy
        msg.orientation.z = qz
        msg.orientation.w = qw
        self.pose_publisher.publish(msg)

    def publish_speed(self, linear_m_s: float, angular_rad_s: float):
        msg = Twist()
        msg.linear.x = float(linear_m_s)
        msg.angular.z = float(angular_rad_s)
        self.speed_publisher.publish(msg)

    def publish_offsets(self, tool_offset: float, object_offset: float):
        msg = Float64MultiArray()
        msg.data = [float(tool_offset), float(object_offset)]
        self.offset_publisher.publish(msg)

    def publish_joint_positions(self, joint_positions, duration_s: float = 0.02):
        if not joint_positions:
            return
        positions = list(joint_positions)
        if len(positions) < len(self.joint_names):
            positions.extend([0.0] * (len(self.joint_names) - len(positions)))
        msg = JointTrajectory()
        msg.joint_names = list(self.joint_names)
        point = JointTrajectoryPoint()
        point.positions = [float(p) for p in positions]
        point.time_from_start = Duration(sec=0, nanosec=int(duration_s * 1e9))
        msg.points.append(point)
        self.joint_command_publisher.publish(msg)
        if self.hardware_available():
            self.hardware_joint_command_publisher.publish(msg)

    @staticmethod
    def _euler_to_quaternion(roll: float, pitch: float, yaw: float):
        cr = math.cos(roll / 2.0)
        sr = math.sin(roll / 2.0)
        cp = math.cos(pitch / 2.0)
        sp = math.sin(pitch / 2.0)
        cy = math.cos(yaw / 2.0)
        sy = math.sin(yaw / 2.0)
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        qw = cr * cp * cy + sr * sp * sy
        return qx, qy, qz, qw


class LabeledSlider(QWidget):
    def __init__(self, spec: SliderSpec, parent=None):
        super().__init__(parent)
        self.scale = 1000.0

        layout = QVBoxLayout(self)
        layout.setContentsMargins(16, 6, 16, 6)

        header = QHBoxLayout()
        self.label = QLabel(spec.label)
        self.label.setObjectName("sliderLabel")
        self.value_label = QLabel(self._format_value(spec.default_mm))
        self.value_label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        self.value_label.setObjectName("valueLabel")
        header.addWidget(self.label)
        header.addStretch(1)
        header.addWidget(self.value_label)
        layout.addLayout(header)

        self.slider = QSlider(Qt.Horizontal)
        self.slider.setRange(spec.minimum_mm, spec.maximum_mm)
        self.slider.setSingleStep(1)
        self.slider.setPageStep(5)
        self.slider.setValue(spec.default_mm)
        self.slider.valueChanged.connect(self._update_value_label)
        layout.addWidget(self.slider)

    def _format_value(self, value_mm: int) -> str:
        return f"{value_mm / self.scale:.3f} m"

    def _update_value_label(self, value_mm: int):
        self.value_label.setText(self._format_value(value_mm))

    def value_m(self) -> float:
        return self.slider.value() / self.scale

    def set_mm(self, value_mm: int):
        self.slider.setValue(value_mm)


class DeltaRobotGui(QMainWindow):
    def __init__(self):
        super().__init__()

        if not rclpy.ok():
            rclpy.init(args=None)

        self.node = DeltaGuiNode()
        # Subscribe to motor init status messages (published by motor_control_node)
        try:
            self.node.create_subscription(
                String,
                self.node.init_status_topic,
                self._on_init_status_msg,
                10,
            )
        except Exception:
            # If subscription cannot be created now, ignore - ros_spin timer will pick it up when node ready
            pass
        self.pending_file_job = None
        self.active_job_type = None
        self.active_job_name = None
        self.stop_requested = False
        self.motor_angles_window = None

        self.setWindowTitle("Delta Robot Control Center")
        self.setMinimumSize(875, 1080)
        self.is_wayland = "wayland" in QGuiApplication.platformName().lower()
        self._drag_position = QPoint()
        self._drag_active = False

        if self.is_wayland:
            self.setAttribute(Qt.WA_TranslucentBackground, True)
            self.setAutoFillBackground(False)
            self.setWindowFlag(Qt.FramelessWindowHint, True)

        self._build_ui()
        self._apply_styles()
        self._connect_services()

        self.status_timer = QTimer(self)
        self.status_timer.timeout.connect(self._refresh_status)
        self.status_timer.start(500)

        self.live_publish_timer = QTimer(self)
        self.live_publish_timer.setSingleShot(True)
        self.live_publish_timer.timeout.connect(self._publish_live_target)

        self.ros_spin_timer = QTimer(self)
        self.ros_spin_timer.timeout.connect(self._spin_ros)
        self.ros_spin_timer.start(10)

        self.process = QProcess(self)
        self.process.setProcessChannelMode(QProcess.MergedChannels)
        self.process.readyReadStandardOutput.connect(self._read_process_output)
        self.process.finished.connect(self._process_finished)
        self.process.errorOccurred.connect(self._process_error)

    def _build_ui(self):
        central = QWidget(self)
        self.setCentralWidget(central)
        central.setObjectName("rootSurface")

        root = QVBoxLayout(central)
        root.setContentsMargins(8, 8, 8, 8)
        root.setSpacing(0)

        shell = QFrame()
        shell.setObjectName("windowShell")
        shell_layout = QVBoxLayout(shell)
        shell_layout.setContentsMargins(10, 10, 10, 10)
        shell_layout.setSpacing(10)

        if self.is_wayland:
            self.window_header = QFrame()
            self.window_header.setObjectName("windowHeader")
            self.window_header.setFixedHeight(42)
            self.window_header.installEventFilter(self)

            header_layout = QHBoxLayout(self.window_header)
            header_layout.setContentsMargins(14, 8, 10, 8)
            header_layout.setSpacing(8)

            header_title = QLabel("Delta Robot Control Center")
            header_title.setObjectName("headerTitle")
            header_subtitle = QLabel("Wayland native shell")
            header_subtitle.setObjectName("headerSubtitle")

            title_col = QVBoxLayout()
            title_col.setContentsMargins(0, 0, 0, 0)
            title_col.setSpacing(0)
            title_col.addWidget(header_title)
            title_col.addWidget(header_subtitle)

            header_layout.addLayout(title_col)
            header_layout.addStretch(1)

            self.minimize_button = QPushButton("▁")
            self.minimize_button.setObjectName("windowControlButton")
            self.minimize_button.setFixedSize(30, 24)
            self.minimize_button.clicked.connect(self.showMinimized)

            self.maximize_button = QPushButton("□")
            self.maximize_button.setObjectName("windowControlButton")
            self.maximize_button.setFixedSize(30, 24)
            self.maximize_button.clicked.connect(self._toggle_maximize)

            self.close_button = QPushButton("✕")
            self.close_button.setObjectName("windowCloseButton")
            self.close_button.setFixedSize(30, 24)
            self.close_button.clicked.connect(self.close)

            header_layout.addWidget(self.minimize_button)
            header_layout.addWidget(self.maximize_button)
            header_layout.addWidget(self.close_button)
            shell_layout.addWidget(self.window_header)

        hero = QFrame()
        hero.setObjectName("heroCard")
        hero_layout = QVBoxLayout(hero)
        hero_layout.setContentsMargins(14, 12, 14, 12)
        hero_layout.setSpacing(6)

        title = QLabel("Delta Robot Control Center")
        title.setObjectName("titleLabel")
        subtitle = QLabel(
            "Live Cartesian control, G-code playback, and JSON task runs in one focused interface."
        )
        subtitle.setObjectName("subtitleLabel")
        hero_layout.addWidget(title)
        hero_layout.addWidget(subtitle)

        # Control Mode Toggle
        mode_box = QHBoxLayout()
        mode_label = QLabel("Control Mode:")
        mode_label.setStyleSheet("font-weight: bold;")
        self.manual_mode_btn = QRadioButton("Manual (Sliders)")
        self.manual_mode_btn.setChecked(True)
        self.task_mode_btn = QRadioButton("Auto (Tasks/G-code)")
        self.manual_mode_btn.toggled.connect(self._on_mode_toggled)
        
        mode_box.addWidget(mode_label)
        mode_box.addWidget(self.manual_mode_btn)
        mode_box.addWidget(self.task_mode_btn)
        mode_box.addStretch(1)
        hero_layout.addLayout(mode_box)

        status_row = QHBoxLayout()
        self.service_indicator = QLabel("Feedback: waiting...")
        self.service_indicator.setObjectName("serviceIndicator")
        self.service_indicator.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)

        self.activity_indicator = QLabel("Idle")
        self.activity_indicator.setObjectName("activityIndicator")
        self.activity_indicator.setAlignment(Qt.AlignRight | Qt.AlignVCenter)

        self.mode_indicator = QLabel("Live publish: on")
        self.mode_indicator.setObjectName("modeIndicator")
        self.mode_indicator.setAlignment(Qt.AlignCenter)

        status_row.addWidget(self.service_indicator)
        status_row.addWidget(self.mode_indicator)
        status_row.addWidget(self.activity_indicator)
        hero_layout.addLayout(status_row)

        self._add_shadow(hero, blur=34, y_offset=8)
        shell_layout.addWidget(hero)

        self.tabs = QTabWidget()
        self.cartesian_tab = self._build_cartesian_tab()
        self.settings_tab = self._build_settings_tab() # NEW TAB
        self.gcode_tab = self._build_gcode_tab()
        self.json_tab = self._build_json_tab()
        
        self.tabs.addTab(self.cartesian_tab, "Cartesian")
        self.tabs.addTab(self.settings_tab, "Settings") # ADDED HERE
        self.tabs.addTab(self.gcode_tab, "G-code")
        self.tabs.addTab(self.json_tab, "JSON Tasks")
        self.tabs.addTab(self._build_console_tab(), "Console")
        
        self.gcode_tab.setEnabled(False)
        self.json_tab.setEnabled(False)
        
        # Connect tab clicks to auto-switch control modes!
        self.tabs.currentChanged.connect(self._on_tab_changed)
        
        shell_layout.addWidget(self.tabs)
        root.addWidget(shell)

    def _on_mode_toggled(self, checked):
        is_manual = self.manual_mode_btn.isChecked()
        self.cartesian_tab.setEnabled(is_manual)
        self.gcode_tab.setEnabled(not is_manual)
        self.json_tab.setEnabled(not is_manual)
        
        if is_manual:
            is_live = self.live_move_checkbox.isChecked()
            self.mode_indicator.setText("Live publish: on" if is_live else "Live publish: off")
            if is_live:
                self._schedule_live_publish(immediate=True)
        else:
            self.mode_indicator.setText("Mode: Auto (Tasks)")
            self.live_publish_timer.stop()

    def eventFilter(self, obj, event):
        if (
            self.is_wayland
            and hasattr(self, "window_header")
            and obj is self.window_header
        ):
            if (
                event.type() == QEvent.MouseButtonPress
                and event.button() == Qt.LeftButton
            ):
                window = self.windowHandle()
                if (
                    window is not None
                    and hasattr(window, "startSystemMove")
                    and window.startSystemMove()
                ):
                    return True
                self._drag_active = True
                self._drag_position = event.globalPos() - self.frameGeometry().topLeft()
                return True
            if (
                event.type() == QEvent.MouseMove
                and self._drag_active
                and event.buttons() & Qt.LeftButton
            ):
                self.move(event.globalPos() - self._drag_position)
                return True
            if event.type() == QEvent.MouseButtonRelease:
                self._drag_active = False
                return True
            if event.type() == QEvent.MouseButtonDblClick:
                self._toggle_maximize()
                return True
        return super().eventFilter(obj, event)

    def _toggle_maximize(self):
        if self.isMaximized():
            self.showNormal()
            if hasattr(self, "maximize_button"):
                self.maximize_button.setText("□")
        else:
            self.showMaximized()
            if hasattr(self, "maximize_button"):
                self.maximize_button.setText("❐")

    def _build_cartesian_tab(self) -> QWidget:
        tab = QWidget()
        layout = QVBoxLayout(tab)
        layout.setContentsMargins(16, 16, 16, 16)
        layout.setSpacing(16)

        # Header / Status Card
        help_box = QFrame()
        help_box.setObjectName("infoCard")
        help_layout = QHBoxLayout(help_box)
        help_layout.setContentsMargins(16, 12, 16, 12)
        help_layout.setSpacing(10)

        hint_col = QVBoxLayout()
        hint = QLabel("Live Manual Control")
        hint.setStyleSheet("font-weight: bold; font-size: 14px; color: #eef5fb;")
        subhint = QLabel("Adjust sliders to command cartesian poses.")
        subhint.setStyleSheet("color: #9aa9b9; font-size: 12px;")
        hint_col.addWidget(hint)
        hint_col.addWidget(subhint)

        self.live_move_checkbox = QCheckBox("Publish Live")
        self.live_move_checkbox.setChecked(True)
        self.live_move_checkbox.toggled.connect(self._on_live_mode_changed)

        help_layout.addLayout(hint_col)
        help_layout.addStretch(1)
        help_layout.addWidget(self.live_move_checkbox)

        self._add_shadow(help_box, blur=24, y_offset=6)
        layout.addWidget(help_box)

        # Sliders Box
        slider_box = QGroupBox("Target Coordinates")
        slider_box.setObjectName("cardBox")
        slider_layout = QVBoxLayout(slider_box)
        slider_layout.setContentsMargins(10, 16, 10, 16)
        slider_layout.setSpacing(12)

        self.x_slider = LabeledSlider(SliderSpec("X", -120, 120, 0))
        self.y_slider = LabeledSlider(SliderSpec("Y", -120, 120, 0))
        self.z_slider = LabeledSlider(SliderSpec("Z", -450, -300, -375))
        
        # Subtle visual separator between Translation and Rotation
        separator = QFrame()
        separator.setFrameShape(QFrame.HLine)
        separator.setStyleSheet("background-color: rgba(255, 255, 255, 0.05); margin: 4px 16px;")
        
        self.tilt_slider = LabeledSlider(SliderSpec("Tilt", -90, 90, 0))
        self.spin_slider = LabeledSlider(SliderSpec("Spin", -180, 180, 0))

        self.x_slider.slider.valueChanged.connect(self._on_cartesian_slider_changed)
        self.y_slider.slider.valueChanged.connect(self._on_cartesian_slider_changed)
        self.z_slider.slider.valueChanged.connect(self._on_cartesian_slider_changed)
        self.tilt_slider.slider.valueChanged.connect(self._on_cartesian_slider_changed)
        self.spin_slider.slider.valueChanged.connect(self._on_cartesian_slider_changed)
        self.x_slider.slider.sliderPressed.connect(self._on_cartesian_slider_pressed)
        self.y_slider.slider.sliderPressed.connect(self._on_cartesian_slider_pressed)
        self.z_slider.slider.sliderPressed.connect(self._on_cartesian_slider_pressed)
        self.tilt_slider.slider.sliderPressed.connect(self._on_cartesian_slider_pressed)
        self.spin_slider.slider.sliderPressed.connect(self._on_cartesian_slider_pressed)

        slider_layout.addWidget(self.x_slider)
        slider_layout.addWidget(self.y_slider)
        slider_layout.addWidget(self.z_slider)
        slider_layout.addWidget(separator)
        slider_layout.addWidget(self.tilt_slider)
        slider_layout.addWidget(self.spin_slider)
        layout.addWidget(slider_box)
        self._add_shadow(slider_box, blur=24, y_offset=6)

        # Target Preview Area
        preview_box = QFrame()
        preview_layout = QVBoxLayout(preview_box)
        preview_layout.setContentsMargins(0,0,0,0)
        self.target_preview = QLabel("Target: x=0.000 m, y=0.000 m, z=-0.180 m, tilt=0.0 deg, spin=0.0 deg")
        self.target_preview.setObjectName("previewLabel")
        self.target_preview.setStyleSheet("color: #89bdf1; font-size: 13px; font-weight: 600; padding: 6px 8px; background: rgba(137, 189, 241, 0.1); border-radius: 6px;")
        self.target_preview.setAlignment(Qt.AlignCenter)
        preview_layout.addWidget(self.target_preview)
        layout.addWidget(preview_box)

        # Action Buttons (New Grid Layout for better UX)
        button_grid = QGridLayout()
        button_grid.setSpacing(12)

        self.send_button = QPushButton("Send Pos")
        self.send_button.setObjectName("primaryButton")
        self.send_button.setMinimumHeight(40)
        self.send_button.clicked.connect(self._send_target_from_sliders)

        self.home_button = QPushButton("Home Pose")
        self.home_button.setObjectName("secondaryButton")
        self.home_button.setMinimumHeight(40)
        self.home_button.clicked.connect(self._home_position)

        self.zero_xy_button = QPushButton("Zero X/Y")
        self.zero_xy_button.setObjectName("secondaryButton")
        self.zero_xy_button.setMinimumHeight(40)
        self.zero_xy_button.clicked.connect(self._zero_xy)

        self.see_motors_button = QPushButton("See Motors")
        self.see_motors_button.setObjectName("secondaryButton")
        self.see_motors_button.setMinimumHeight(40)
        self.see_motors_button.clicked.connect(self._open_motor_angles_window)

        button_grid.addWidget(self.send_button, 0, 0, 1, 2)
        button_grid.addWidget(self.home_button, 1, 0)
        button_grid.addWidget(self.zero_xy_button, 1, 1)
        button_grid.addWidget(self.see_motors_button, 2, 0, 1, 2)
        layout.addLayout(button_grid)

        # Feedback
        self.feedback_label = QLabel("Ready.")
        self.feedback_label.setObjectName("feedbackLabel")
        self.feedback_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.feedback_label)

        layout.addItem(QSpacerItem(20, 20, QSizePolicy.Minimum, QSizePolicy.Expanding))
        return tab

    def _on_tab_changed(self, index: int):
        """Automatically switch control modes based on the clicked tab."""
        tab_name = self.tabs.tabText(index)
        
        if tab_name in ["Cartesian", "Settings"]:
            self.manual_mode_btn.setChecked(True)
        elif tab_name in ["G-code", "JSON Tasks"]:
            self.task_mode_btn.setChecked(True)
            
    def _build_settings_tab(self) -> QWidget:
        tab = QWidget()
        layout = QVBoxLayout(tab)
        layout.setContentsMargins(16, 16, 16, 16)
        layout.setSpacing(16)

        info = QLabel("Configure physical offsets, movement limits, and kinematics.")
        info.setObjectName("hintLabel")
        info.setStyleSheet("padding: 0 0 8px 0; color: #9aa9b9; font-size: 13px;")
        layout.addWidget(info)

        # -- 1. Offsets Box --
        offset_box = QGroupBox("Physical Offsets")
        offset_box.setObjectName("cardBox")
        offset_layout = QFormLayout(offset_box)
        offset_layout.setContentsMargins(14, 20, 14, 14)
        offset_layout.setSpacing(12)

        self.tool_offset_spin = QDoubleSpinBox()
        self.tool_offset_spin.setDecimals(3)
        self.tool_offset_spin.setRange(-0.5, 0.5)
        self.tool_offset_spin.setSingleStep(0.001)
        self.tool_offset_spin.setValue(0.0)
        self.tool_offset_spin.setSuffix(" m")
        self.tool_offset_spin.valueChanged.connect(self._publish_offsets)

        self.object_offset_spin = QDoubleSpinBox()
        self.object_offset_spin.setDecimals(3)
        self.object_offset_spin.setRange(-0.5, 0.5)
        self.object_offset_spin.setSingleStep(0.001)
        self.object_offset_spin.setValue(0.0)
        self.object_offset_spin.setSuffix(" m")
        self.object_offset_spin.valueChanged.connect(self._publish_offsets)

        offset_layout.addRow("Tool Z-Offset (from wrist):", self.tool_offset_spin)
        offset_layout.addRow("Object Z-Offset (from table):", self.object_offset_spin)
        layout.addWidget(offset_box)
        self._add_shadow(offset_box, blur=20, y_offset=4)

        # -- 2. Speeds Box --
        speed_box = QGroupBox("Default Movement Speeds")
        speed_box.setObjectName("cardBox")
        speed_layout = QFormLayout(speed_box)
        speed_layout.setContentsMargins(14, 20, 14, 14)
        speed_layout.setSpacing(12)

        self.linear_speed_spin = QDoubleSpinBox()
        self.linear_speed_spin.setDecimals(3)
        self.linear_speed_spin.setRange(0.001, 2.0)
        self.linear_speed_spin.setSingleStep(0.01)
        self.linear_speed_spin.setValue(0.3)
        self.linear_speed_spin.setSuffix(" m/s")
        self.linear_speed_spin.valueChanged.connect(self._publish_speed_settings)

        self.angular_speed_spin = QDoubleSpinBox()
        self.angular_speed_spin.setDecimals(3)
        self.angular_speed_spin.setRange(0.01, 10.0)
        self.angular_speed_spin.setSingleStep(0.05)
        self.angular_speed_spin.setValue(1.0)
        self.angular_speed_spin.setSuffix(" rad/s")
        self.angular_speed_spin.valueChanged.connect(self._publish_speed_settings)

        speed_layout.addRow("Max Linear Speed:", self.linear_speed_spin)
        speed_layout.addRow("Max Angular Speed:", self.angular_speed_spin)
        layout.addWidget(speed_box)
        self._add_shadow(speed_box, blur=20, y_offset=4)

        # -- 3. Kinematics Box --
        orientation_box = QGroupBox("Kinematics & Control")
        orientation_box.setObjectName("cardBox")
        orientation_layout = QVBoxLayout(orientation_box)
        orientation_layout.setContentsMargins(14, 20, 14, 14)
        orientation_layout.setSpacing(10)

        self.spin_enable_checkbox = QCheckBox("Enable 5-DOF Orientation (Tilt + Spin)")
        self.spin_enable_checkbox.setChecked(True)
        self.spin_enable_checkbox.toggled.connect(self._on_orientation_enabled_toggled)

        kinematics_hint = QLabel("When disabled, the IK solver will force orientation to 0 degrees.")
        kinematics_hint.setObjectName("hintLabel")
        kinematics_hint.setStyleSheet("padding: 0 0 0 28px; color: #7a8fa3; font-size: 11px;")

        orientation_layout.addWidget(self.spin_enable_checkbox)
        orientation_layout.addWidget(kinematics_hint)
        layout.addWidget(orientation_box)
        self._add_shadow(orientation_box, blur=20, y_offset=4)

        layout.addItem(QSpacerItem(20, 20, QSizePolicy.Minimum, QSizePolicy.Expanding))
        return tab
    def _build_gcode_tab(self) -> QWidget:
        tab = QWidget()
        layout = QVBoxLayout(tab)
        layout.setContentsMargins(12, 12, 12, 12)
        layout.setSpacing(14)

        description = QLabel(
            "Pick a G-code file and run it through velocity_pub/delta_gcode_interpreter.py."
        )
        description.setObjectName("hintLabel")
        description.setStyleSheet("padding: 8px 6px; color: #9aa9b9; font-size: 12px;")
        layout.addWidget(description)

        file_box = QGroupBox("File Selection")
        file_box.setObjectName("cardBox")
        file_layout = QHBoxLayout(file_box)
        file_layout.setContentsMargins(14, 18, 14, 14)
        file_layout.setSpacing(10)

        self.gcode_path = QLineEdit()
        self.gcode_path.setPlaceholderText("Select a .gcode or .nc file")
        self.gcode_path.setMinimumHeight(36)
        gcode_browse = QPushButton("Browse")
        gcode_browse.setObjectName("secondaryButton")
        gcode_browse.setMaximumWidth(100)
        gcode_browse.setMinimumHeight(36)
        gcode_browse.clicked.connect(self._browse_gcode_file)

        file_layout.addWidget(self.gcode_path)
        file_layout.addWidget(gcode_browse)
        layout.addWidget(file_box)
        self._add_shadow(file_box, blur=20, y_offset=4)

        settings_box = QGroupBox("Execution Settings")
        settings_box.setObjectName("cardBox")
        settings_layout = QVBoxLayout(settings_box)
        settings_layout.setContentsMargins(14, 20, 14, 14)
        settings_layout.setSpacing(16)

        self.gcode_loop_checkbox = QCheckBox("Loop this file after it finishes")
        self.gcode_loop_checkbox.setChecked(False)
        self.gcode_loop_checkbox.toggled.connect(self._on_loop_toggled)
        settings_layout.addWidget(self.gcode_loop_checkbox)

        note = QLabel("Units and rate are driven by the G-code file itself (G20/G21, F commands).")
        note.setObjectName("hintLabel")
        note.setStyleSheet("padding: 6px 6px; color: #9aa9b9; font-size: 11px;")
        settings_layout.addWidget(note)

        layout.addWidget(settings_box)
        self._add_shadow(settings_box, blur=22, y_offset=5)

        run_row = QHBoxLayout()
        run_row.setSpacing(10)
        self.run_gcode_button = QPushButton("Run G-code")
        self.run_gcode_button.setObjectName("primaryButton")
        self.run_gcode_button.setMinimumHeight(36)
        self.run_gcode_button.clicked.connect(self._run_gcode_file)
        self.gcode_stop_button = QPushButton("Stop")
        self.gcode_stop_button.setObjectName("secondaryButton")
        self.gcode_stop_button.setMinimumHeight(36)
        self.gcode_stop_button.setMaximumWidth(120)
        self.gcode_stop_button.clicked.connect(self._stop_process)
        run_row.addWidget(self.run_gcode_button)
        run_row.addWidget(self.gcode_stop_button)
        run_row.addStretch(1)
        layout.addLayout(run_row)

        layout.addItem(QSpacerItem(20, 20, QSizePolicy.Minimum, QSizePolicy.Expanding))
        return tab

    def _build_json_tab(self) -> QWidget:
        tab = QWidget()
        layout = QVBoxLayout(tab)
        layout.setContentsMargins(12, 12, 12, 12)
        layout.setSpacing(14)

        description = QLabel(
            "Pick a JSON task list and run it through velocity_pub/task_sequencer.py."
        )
        description.setObjectName("hintLabel")
        description.setStyleSheet("padding: 8px 6px; color: #9aa9b9; font-size: 12px;")
        layout.addWidget(description)

        file_box = QGroupBox("File Selection")
        file_box.setObjectName("cardBox")
        file_layout = QHBoxLayout(file_box)
        file_layout.setContentsMargins(14, 18, 14, 14)
        file_layout.setSpacing(10)

        self.json_path = QLineEdit()
        self.json_path.setPlaceholderText("Select a task .json file")
        self.json_path.setMinimumHeight(36)
        json_browse = QPushButton("Browse")
        json_browse.setObjectName("secondaryButton")
        json_browse.setMaximumWidth(100)
        json_browse.setMinimumHeight(36)
        json_browse.clicked.connect(self._browse_json_file)

        file_layout.addWidget(self.json_path)
        file_layout.addWidget(json_browse)
        layout.addWidget(file_box)
        self._add_shadow(file_box, blur=20, y_offset=4)

        settings_box = QGroupBox("Execution Settings")
        settings_box.setObjectName("cardBox")
        settings_layout = QVBoxLayout(settings_box)
        settings_layout.setContentsMargins(14, 20, 14, 14)
        settings_layout.setSpacing(16)

        self.json_loop_checkbox = QCheckBox("Loop this task list after it finishes")
        self.json_loop_checkbox.setChecked(False)
        self.json_loop_checkbox.toggled.connect(self._on_loop_toggled)
        settings_layout.addWidget(self.json_loop_checkbox)

        note = QLabel("Task timing is controlled by the JSON actions (duration/seconds).")
        note.setObjectName("hintLabel")
        note.setStyleSheet("padding: 6px 6px; color: #9aa9b9; font-size: 11px;")
        settings_layout.addWidget(note)

        layout.addWidget(settings_box)
        self._add_shadow(settings_box, blur=22, y_offset=5)

        run_row = QHBoxLayout()
        run_row.setSpacing(10)
        self.run_json_button = QPushButton("Run JSON")
        self.run_json_button.setObjectName("primaryButton")
        self.run_json_button.setMinimumHeight(36)
        self.run_json_button.clicked.connect(self._run_json_file)
        self.json_stop_button = QPushButton("Stop")
        self.json_stop_button.setObjectName("secondaryButton")
        self.json_stop_button.setMinimumHeight(36)
        self.json_stop_button.setMaximumWidth(120)
        self.json_stop_button.clicked.connect(self._stop_process)
        run_row.addWidget(self.run_json_button)
        run_row.addWidget(self.json_stop_button)
        run_row.addStretch(1)
        layout.addLayout(run_row)

        layout.addItem(QSpacerItem(20, 20, QSizePolicy.Minimum, QSizePolicy.Expanding))
        return tab


    def _build_console_tab(self) -> QWidget:
        tab = QWidget()
        layout = QVBoxLayout(tab)
        layout.setContentsMargins(12, 12, 12, 12)
        layout.setSpacing(10)

        info = QLabel("Process output and errors appear here.")
        info.setObjectName("hintLabel")
        info.setStyleSheet("padding: 8px 6px; color: #9aa9b9; font-size: 12px;")
        layout.addWidget(info)

        self.console = QPlainTextEdit()
        self.console.setReadOnly(True)
        self.console.setObjectName("consoleBox")
        self.console.setMinimumHeight(300)
        layout.addWidget(self.console)

        clear_row = QHBoxLayout()
        clear_row.setSpacing(8)
        clear_row.addStretch(1)
        clear_button = QPushButton("Clear Console")
        clear_button.setObjectName("secondaryButton")
        clear_button.setMinimumHeight(36)
        clear_button.setMaximumWidth(140)
        clear_button.clicked.connect(self.console.clear)
        clear_row.addWidget(clear_button)
        layout.addLayout(clear_row)

        return tab

    def _apply_styles(self):
        self.setStyleSheet(
            """
            QWidget {
                background: transparent;
                color: #e7eef7;
                font-family: "SF Pro Display Nerd Font", "SF Pro Text Nerd Font", "SF Pro Display", "SF Pro Text", "Noto Sans", "DejaVu Sans", sans-serif;
                font-size: 13px;
            }
            QWidget#rootSurface {
                background: transparent;
            }
            QFrame#windowShell {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:1,
                    stop:0 #0a1017, stop:0.55 #101b27, stop:1 #0b1320);
                border: 1px solid rgba(255, 255, 255, 0.08);
                border-radius: 30px;
            }
            QFrame#windowHeader {
                background: rgba(255, 255, 255, 0.03);
                border: 1px solid rgba(255, 255, 255, 0.05);
                border-radius: 18px;
            }
            QLabel#headerTitle {
                color: #f4f8fc;
                font-size: 14px;
                font-weight: 700;
                letter-spacing: 0.2px;
                font-family: "SF Pro Display Nerd Font", "SF Pro Display", "SF Pro Text Nerd Font", "Noto Sans", sans-serif;
            }
            QLabel#headerSubtitle {
                color: #90a2b5;
                font-size: 11px;
                font-family: "SF Pro Text Nerd Font", "SF Pro Text", "Noto Sans", sans-serif;
            }
            QPushButton#windowControlButton,
            QPushButton#windowCloseButton {
                border: 1px solid rgba(255, 255, 255, 0.10);
                border-radius: 8px;
                padding: 0;
                font-weight: 700;
                background: rgba(255, 255, 255, 0.05);
                color: #e7eef7;
            }
            QPushButton#windowControlButton:hover {
                background: rgba(255, 255, 255, 0.12);
            }
            QPushButton#windowCloseButton {
                background: rgba(186, 74, 74, 0.18);
                color: #ffd9d9;
                border: 1px solid rgba(186, 74, 74, 0.28);
            }
            QPushButton#windowCloseButton:hover {
                background: rgba(220, 92, 92, 0.30);
            }
            QFrame#heroCard,
            QFrame#infoCard,
            QGroupBox#cardBox,
            QGroupBox {
                background: rgba(16, 24, 35, 0.92);
                border: 1px solid rgba(255, 255, 255, 0.10);
                border-radius: 10px;
                margin-top: 8px;
            }
            QFrame#heroCard:hover,
            QFrame#infoCard:hover,
            QGroupBox#cardBox:hover {
                border-color: rgba(255, 255, 255, 0.14);
                background: rgba(16, 24, 35, 0.96);
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 14px;
                top: 2px;
                padding: 0 8px;
                color: #e8f0f8;
                font-weight: 700;
                font-size: 13px;
            }
            QLabel#titleLabel {
                font-size: 27px;
                font-weight: 800;
                letter-spacing: 0.2px;
                color: #f4f8fc;
                font-family: "SF Pro Display Nerd Font", "SF Pro Display", "SF Pro Text Nerd Font", "Noto Sans", sans-serif;
            }
            QLabel#subtitleLabel {
                color: #98a9ba;
                margin-bottom: 2px;
                font-size: 13px;
                font-family: "SF Pro Text Nerd Font", "SF Pro Text", "Noto Sans", sans-serif;
            }
            QLabel#hintLabel {
                color: #9aa9b9;
                font-size: 12px;
                padding: 10px 8px;
                line-height: 1.5;
                background: transparent;
            }
            QLabel#serviceIndicator,
            QLabel#activityIndicator,
            QLabel#modeIndicator {
                color: #d7e2ec;
                padding: 7px 12px;
                background: rgba(255, 255, 255, 0.04);
                border: 1px solid rgba(255, 255, 255, 0.08);
                border-radius: 999px;
                font-size: 12px;
                font-weight: 500;
            }
            QTabWidget::pane {
                border: 1px solid rgba(255, 255, 255, 0.08);
                border-radius: 10px;
                top: -1px;
                background: rgba(16, 24, 35, 0.88);
            }
            QTabBar::tab {
                background: rgba(255, 255, 255, 0.05);
                color: #b8c6d6;
                padding: 11px 18px;
                margin-right: 2px;
                border-top-left-radius: 8px;
                border-top-right-radius: 8px;
                min-width: 100px;
                font-weight: 500;
                font-family: "SF Pro Text Nerd Font", "SF Pro Text", "Noto Sans", sans-serif;
            }
            QTabBar::tab:hover {
                background: rgba(255, 255, 255, 0.08);
                color: #d4dfe9;
            }
            QTabBar::tab:selected {
                background: rgba(58, 141, 222, 0.35);
                color: #f7fbff;
                font-weight: 600;
            }
            QLabel#sliderLabel {
                font-weight: 700;
                color: #eef5fb;
            }
            QLabel#valueLabel {
                color: #d7ecff;
                min-width: 104px;
                min-height: 22px;
                padding: 2px 7px;
                margin-right: 2px;
                background: rgba(42, 108, 176, 0.42);
                border: 1px solid rgba(96, 169, 238, 0.60);
                border-radius: 8px;
                font-size: 11px;
                font-weight: 600;
                font-family: "SF Pro Text Nerd Font", "SF Pro Text", "Noto Sans", "DejaVu Sans", sans-serif;
            }
            QSlider::groove:horizontal {
                border: none;
                height: 11px;
                background: #16202c;
                border-radius: 6px;
            }
            QSlider::sub-page:horizontal {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                    stop:0 #4b9ef0, stop:1 #2b76d1);
                border-radius: 6px;
            }
            QSlider::handle:horizontal {
                background: #f7fbff;
                border: 2px solid #5ba3ec;
                width: 20px;
                margin: -6px 0;
                border-radius: 10px;
            }
            QLineEdit, QComboBox, QDoubleSpinBox, QTextEdit {
                background: rgba(8, 14, 21, 0.85);
                border: 1px solid rgba(255, 255, 255, 0.12);
                border-radius: 8px;
                padding: 10px 12px;
                color: #e7eef7;
                selection-background-color: rgba(58, 141, 222, 0.40);
                font-size: 13px;
                font-family: "SF Pro Text Nerd Font", "SF Pro Text", "Noto Sans", sans-serif;
            }
            QLineEdit:focus, QComboBox:focus, QDoubleSpinBox:focus, QTextEdit:focus {
                background: rgba(8, 14, 21, 0.95);
                border: 2px solid rgba(58, 141, 222, 0.50);
            }
            QComboBox::drop-down {
                border: none;
                padding-right: 8px;
                subcontrol-position: center right;
                subcontrol-origin: padding;
                width: 24px;
            }
            QComboBox::down-arrow {
                image: none;
                color: #89bdf1;
                width: 8px;
                height: 8px;
            }
            QDoubleSpinBox::up-button, QDoubleSpinBox::down-button {
                width: 24px;
                border: none;
                background: rgba(58, 141, 222, 0.12);
                border-left: 1px solid rgba(255, 255, 255, 0.08);
            }
            QDoubleSpinBox::up-button:hover, QDoubleSpinBox::down-button:hover {
                background: rgba(58, 141, 222, 0.20);
            }
            QPlainTextEdit#consoleBox {
                background: rgba(6, 11, 18, 0.95);
                border: 1px solid rgba(255, 255, 255, 0.10);
                border-radius: 8px;
                font-family: "FiraCode Nerd Font", "Fira Code", "SFMono Nerd Font", "DejaVu Sans Mono", "Courier New", monospace;
                font-size: 11px;
                color: #a0b5c7;
                padding: 8px;
            }
            QPlainTextEdit#consoleBox:focus {
                border: 1px solid rgba(58, 141, 222, 0.30);
                background: rgba(6, 11, 18, 0.98);
            }
            QPushButton {
                border: none;
                border-radius: 14px;
                padding: 12px 16px;
                font-weight: 700;
                color: #e9f1f8;
                background: rgba(255, 255, 255, 0.08);
                font-family: "SF Pro Text Nerd Font", "SF Pro Text", "Noto Sans", sans-serif;
            }
            QPushButton:hover {
                background: rgba(255, 255, 255, 0.14);
            }
            QPushButton:pressed {
                background: rgba(255, 255, 255, 0.18);
            }
            QPushButton#primaryButton {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                    stop:0 #4b9ef0, stop:1 #2b76d1);
                color: white;
                padding: 14px 18px;
                font-weight: 600;
            }
            QPushButton#primaryButton:hover {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                    stop:0 #60adef, stop:1 #3b8ae0);
            }
            QPushButton#secondaryButton {
                background: rgba(32, 49, 66, 0.70);
                padding: 12px 16px;
                font-weight: 500;
            }
            QPushButton#secondaryButton:hover {
                background: rgba(32, 49, 66, 0.95);
            }
            QPushButton#modeButton {
                background: rgba(255, 152, 0, 0.30);
                color: #ffc857;
                border: 1px solid rgba(255, 152, 0, 0.50);
                font-weight: 700;
                padding: 10px 18px;
            }
            QPushButton#modeButton:hover {
                background: rgba(255, 152, 0, 0.45);
                border: 1px solid rgba(255, 152, 0, 0.70);
            }
            QLabel#feedbackLabel {
                color: #c1cfdb;
                padding: 10px 2px;
                line-height: 1.4;
                font-size: 12px;
                font-family: "SF Pro Text Nerd Font", "SF Pro Text", "Noto Sans", sans-serif;
            }
            QCheckBox {
                color: #d6e1eb;
                spacing: 10px;
                font-weight: 500;
                padding: 4px 0;
            }
            QCheckBox::indicator {
                width: 18px;
                height: 18px;
            }
            QCheckBox::indicator:unchecked {
                background: rgba(255, 255, 255, 0.08);
                border: 1px solid rgba(255, 255, 255, 0.15);
                border-radius: 4px;
            }
            QCheckBox::indicator:checked {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                    stop:0 #4b9ef0, stop:1 #2b76d1);
                border: 1px solid rgba(58, 141, 222, 0.6);
                border-radius: 4px;
            }
            QCheckBox#loopCheckbox {
                margin-top: 4px;
            }
            """
        )

    def _connect_services(self):
        self._refresh_status()
        self._publish_speed_settings()

    def _spin_ros(self):
        if rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.0)

    def _on_init_status_msg(self, msg):
        """ROS callback for initialization status messages from motor_control_node.
        Schedule a GUI warning dialog on the main thread.
        """
        text = getattr(msg, "data", str(msg))
        QTimer.singleShot(0, lambda: QMessageBox.warning(self, "Motor Init Warning", text))

    def _refresh_status(self):
        now = time.time()
        age_s = None
        if self.node.latest_feedback_time is not None:
            age_s = max(0.0, now - self.node.latest_feedback_time)

        if age_s is not None and age_s < 1.0:
            self.service_indicator.setText(
                f"Feedback: active ({age_s:.2f}s ago)"
            )
            self.service_indicator.setStyleSheet(
                "color: #bdf3c8; background: rgba(61, 156, 85, 0.16); border: 1px solid rgba(61, 156, 85, 0.28);"
            )
        else:
            self.service_indicator.setText(
                f"Feedback: waiting ({self.node.joint_state_topic})"
            )
            self.service_indicator.setStyleSheet(
                "color: #ffd7a8; background: rgba(159, 108, 29, 0.18); border: 1px solid rgba(159, 108, 29, 0.26);"
            )

        if self.node.sim_mode:
            self.activity_indicator.setText("Sim mode")

    def _on_loop_toggled(self, checked: bool):
        sender = self.sender()
        if sender is getattr(self, "gcode_loop_checkbox", None):
            self.feedback_label.setText(
                "G-code loop enabled." if checked else "G-code loop disabled."
            )
            return
        if sender is getattr(self, "json_loop_checkbox", None):
            self.feedback_label.setText(
                "JSON loop enabled." if checked else "JSON loop disabled."
            )
            return
        self.feedback_label.setText("Loop setting updated.")

    def _publish_speed_settings(self):
        if not hasattr(self, "linear_speed_spin"):
            return
        self.node.publish_speed(
            self.linear_speed_spin.value(),
            self.angular_speed_spin.value(),
        )

    def _publish_offsets(self):
        tool_offset = self.tool_offset_spin.value()
        object_offset = self.object_offset_spin.value()
        self.node.publish_offsets(tool_offset, object_offset)

    def _on_live_mode_changed(self, checked: bool):
        if checked:
            self.mode_indicator.setText("Live publish: on")
            self.feedback_label.setText(
                "Live publish enabled: slider changes send pose updates."
            )
            self._schedule_live_publish()
        else:
            self.live_publish_timer.stop()
            self.mode_indicator.setText("Live publish: off")
            self.feedback_label.setText(
                "Live publish disabled: use Send Pos to move."
            )

    def _on_cartesian_slider_pressed(self):
        if self.live_move_checkbox.isChecked():
            self._schedule_live_publish(immediate=True)

    def _on_orientation_enabled_toggled(self, checked: bool):
        self.tilt_slider.setEnabled(checked)
        self.spin_slider.setEnabled(checked)
        if not checked:
            self.feedback_label.setText(
                "Orientation disabled: tilt and spin are forced to 0."
            )
        if self.live_move_checkbox.isChecked():
            self._schedule_live_publish(immediate=True)

    def _effective_orientation_rad(self):
        if not self.spin_enable_checkbox.isChecked():
            return 0.0, 0.0
        tilt_rad = math.radians(self.tilt_slider.slider.value())
        spin_rad = math.radians(self.spin_slider.slider.value())
        return tilt_rad, spin_rad

    def _cartesian_target(self):
        x_m = self.x_slider.value_m()
        y_m = self.y_slider.value_m()
        z_m = self.z_slider.value_m()
        tilt_rad, spin_rad = self._effective_orientation_rad()
        return x_m, y_m, z_m, tilt_rad, spin_rad

    def _on_cartesian_slider_changed(self, _value: int):
        if self.live_move_checkbox.isChecked():
            self._schedule_live_publish(immediate=True)

    def _schedule_live_publish(self, immediate: bool = False):
        self.live_publish_timer.stop()
        if immediate:
            self._publish_live_target()
        else:
            self.live_publish_timer.start(40)

    def _publish_live_target(self):
        x, y, z, tilt, spin = self._cartesian_target()
        self.target_preview.setText(
            f"Target: x={x:.3f} m, y={y:.3f} m, z={z:.3f} m, "
            f"tilt={math.degrees(tilt):.1f} deg, spin={math.degrees(spin):.1f} deg"
        )
        self.feedback_label.setText(
            f"Publishing live pose: x={x:.3f} m, y={y:.3f} m, z={z:.3f} m, "
            f"tilt={math.degrees(tilt):.1f} deg, spin={math.degrees(spin):.1f} deg"
        )
        self.node.publish_pose(x, y, z, tilt, spin)

    def _send_target_from_sliders(self, silent: bool = False):
        x, y, z, tilt, spin = self._cartesian_target()
        self.target_preview.setText(
            f"Target: x={x:.3f} m, y={y:.3f} m, z={z:.3f} m, "
            f"tilt={math.degrees(tilt):.1f} deg, spin={math.degrees(spin):.1f} deg"
        )
        self.feedback_label.setText(
            f"Sending pose: x={x:.3f} m, y={y:.3f} m, z={z:.3f} m, "
            f"tilt={math.degrees(tilt):.1f} deg, spin={math.degrees(spin):.1f} deg"
        )
        self.node.publish_pose(x, y, z, tilt, spin)

    def _home_position(self):
        self.x_slider.set_mm(0)
        self.y_slider.set_mm(0)
        self.z_slider.set_mm(-375)
        self.tilt_slider.set_mm(0)
        self.spin_slider.set_mm(0)
        self._send_target_from_sliders(silent=False)

    def _zero_xy(self):
        self.x_slider.set_mm(0)
        self.y_slider.set_mm(0)
        self.feedback_label.setText("X/Y reset to zero.")

    def _browse_gcode_file(self):
        file_path, _ = QFileDialog.getOpenFileName(
            self,
            "Select G-code file",
            "",
            "G-code Files (*.gcode *.nc *.tap *.txt);;All Files (*)",
        )
        if file_path:
            self.gcode_path.setText(file_path)

    def _browse_json_file(self):
        file_path, _ = QFileDialog.getOpenFileName(
            self,
            "Select JSON task file",
            "",
            "JSON Files (*.json);;All Files (*)",
        )
        if file_path:
            self.json_path.setText(file_path)

    def _run_gcode_file(self):
        file_path = self.gcode_path.text().strip()
        if not file_path:
            QMessageBox.warning(self, "Missing file", "Select a G-code file first.")
            return

        script_path = self._get_task_script_path("delta_gcode_interpreter.py")
        if not os.path.exists(script_path):
            QMessageBox.warning(self, "Missing script", f"Not found: {script_path}")
            return

        args = [script_path, file_path]
        if not self.gcode_loop_checkbox.isChecked():
            args.append("--once")
        self._run_external_process("G-code", sys.executable, args)

    def _run_json_file(self):
        file_path = self.json_path.text().strip()
        if not file_path:
            QMessageBox.warning(self, "Missing file", "Select a JSON task file first.")
            return

        script_path = self._get_task_script_path("task_sequencer.py")
        if not os.path.exists(script_path):
            QMessageBox.warning(self, "Missing script", f"Not found: {script_path}")
            return

        args = [script_path, file_path]
        if self.json_loop_checkbox.isChecked():
            args.append("--loop")
        self._run_external_process("JSON", sys.executable, args)

    def _get_task_script_path(self, script_name: str) -> str:
        """Intelligently find the script path in the delta_tasks package."""
        # 1. Try standard ROS 2 'src' workspace layout
        src_path = os.path.expanduser(f"~/major_project_ws/src/delta_tasks/delta_tasks/{script_name}")
        if os.path.exists(src_path):
            return src_path
            
        # 2. Try flat workspace layout (no 'src' folder)
        flat_path = os.path.expanduser(f"~/major_project_ws/delta_tasks/delta_tasks/{script_name}")
        if os.path.exists(flat_path):
            return flat_path
            
        # Fallback (will trigger the 'Not found' warning in the GUI, but print the correct expected path)
        return src_path

    def _velocity_pub_script_path(self, script_name: str) -> str:
        base_dir = os.path.abspath(
            os.path.join(os.path.dirname(__file__), "..", "..", "velocity_pub", "scripts")
        )
        return os.path.join(base_dir, script_name)

    def _run_external_process(self, label: str, program: str, args: list[str]):
        if self.process.state() != QProcess.NotRunning:
            QMessageBox.information(self, "Busy", "Another command is already running.")
            return

        self.pending_file_job = (label, program, args)
        self.stop_requested = False
        self.active_job_type = "file"
        self.active_job_name = label
        self.console.appendPlainText(f"$ {program} {' '.join(args)}")
        self.activity_indicator.setText(f"Running {label}...")
        self._append_console_line(f"Starting {label} command")
        self.process.start(program, args)
        if not self.process.waitForStarted(3000):
            self.activity_indicator.setText("Idle")
            self._append_console_line("Failed to start command")
            self.pending_file_job = None

    def _stop_process(self):
        self.stop_requested = True
        self.pending_file_job = None
        if hasattr(self, "gcode_loop_checkbox"):
            self.gcode_loop_checkbox.setChecked(False)
        if hasattr(self, "json_loop_checkbox"):
            self.json_loop_checkbox.setChecked(False)
        if self.process.state() == QProcess.NotRunning:
            return
        self._append_console_line("Stopping command")
        self.process.terminate()
        if not self.process.waitForFinished(1000):
            self.process.kill()

    def _read_process_output(self):
        data = bytes(self.process.readAllStandardOutput()).decode(
            "utf-8", errors="replace"
        )
        if data:
            self._append_console_line(data.rstrip())

    def _process_finished(self, exit_code: int, exit_status):
        status_text = "finished" if exit_code == 0 else f"exited with code {exit_code}"
        self.activity_indicator.setText(f"Idle ({status_text})")
        self._append_console_line(f"Command {status_text}")
        loop_enabled = False
        if self.pending_file_job is not None:
            label = self.pending_file_job[0]
            if label == "G-code" and hasattr(self, "gcode_loop_checkbox"):
                loop_enabled = self.gcode_loop_checkbox.isChecked()
            elif label == "JSON" and hasattr(self, "json_loop_checkbox"):
                loop_enabled = self.json_loop_checkbox.isChecked()

        if (
            loop_enabled
            and not self.stop_requested
            and self.pending_file_job is not None
        ):
            label, program, args = self.pending_file_job
            QTimer.singleShot(
                100, lambda: self._run_external_process(label, program, args)
            )

    def _process_error(self, error):
        self.activity_indicator.setText("Idle")
        self._append_console_line(f"Process error: {error}")

    def _append_console_line(self, text: str):
        for line in text.splitlines():
            if line.strip():
                self.console.appendPlainText(line)

    def _add_shadow(self, widget, blur: int = 28, y_offset: int = 6):
        effect = QGraphicsDropShadowEffect(widget)
        effect.setBlurRadius(blur)
        effect.setOffset(0, y_offset)
        effect.setColor(Qt.black)
        widget.setGraphicsEffect(effect)

    def _open_motor_angles_window(self):
        if self.motor_angles_window is None:
            self.motor_angles_window = MotorAnglesWindow(
                self.node.joint_names,
                self.node.joint_state_topic,
                self,
            )
            self.motor_angles_window.setAttribute(Qt.WA_DeleteOnClose, True)
            self.motor_angles_window.destroyed.connect(
                lambda *_: setattr(self, "motor_angles_window", None)
            )

        self.motor_angles_window.show()
        self.motor_angles_window.raise_()
        self.motor_angles_window.activateWindow()

    def closeEvent(self, event):
        self.status_timer.stop()
        self.live_publish_timer.stop()
        self.ros_spin_timer.stop()
        if self.process.state() != QProcess.NotRunning:
            self.process.kill()
        self.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        event.accept()


def main():
    if (
        os.environ.get("WAYLAND_DISPLAY")
        and os.environ.get("QT_QPA_PLATFORM", "").strip() == ""
    ):
        os.environ["QT_QPA_PLATFORM"] = "wayland"
    app = QApplication(sys.argv)
    gui = DeltaRobotGui()
    gui.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
