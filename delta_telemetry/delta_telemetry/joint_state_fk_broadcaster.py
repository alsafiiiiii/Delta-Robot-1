#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from geometry_msgs.msg import TransformStamped
import tf2_ros
import math
from delta_kinematics.delta_ik import DeltaIK

class JointStateFKBroadcaster(Node):
    def __init__(self):
        super().__init__("joint_state_fk_broadcaster")
        self.get_logger().info("Joint State FK Broadcaster Started")

        # IK/FK Solver
        self.ik = DeltaIK()

        # TF Broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Subscriptions
        self.joint_state_sub = self.create_subscription(
            JointState, "/joint_states", self.joint_state_callback, 10
        )
        self.command_sub = self.create_subscription(
            JointTrajectory, "/delta/joint_commands", self.command_callback, 10
        )

        # Parameters for offsets (matching 5DOF controller)
        self.declare_parameter("ee_to_tilt_axis_offset_m", -0.057625)
        self.declare_parameter("tilt_axis_to_tool_tip_offset_m", -0.028385)
        self.declare_parameter("tool_tip_to_object_center_offset_m", -0.0200)

    def _get_tcp_from_joints(self, t1, t2, t3, tilt, spin):
        """Calculates TCP position from joint angles using FK and offsets."""
        try:
            # 1. Solve base FK (End-effector wrist)
            # DeltaIK expects degrees
            wrist_x, wrist_y, wrist_z = self.ik.forward(
                math.degrees(t1), math.degrees(t2), math.degrees(t3)
            )

            # 2. Apply offsets to get TCP
            static_drop = self.get_parameter("ee_to_tilt_axis_offset_m").value
            rotating_length = (
                self.get_parameter("tilt_axis_to_tool_tip_offset_m").value
                + self.get_parameter("tool_tip_to_object_center_offset_m").value
            )

            # Reversing the logic from 5DOF controller's IK compensation
            # Wrist position was:
            # wrist_y = tcp_y + (rotating_length * sin(tilt))
            # wrist_z = tcp_z - static_drop - (rotating_length * cos(tilt))
            
            # Therefore:
            tcp_x = wrist_x
            tcp_y = wrist_y - (rotating_length * math.sin(tilt))
            tcp_z = wrist_z + static_drop + (rotating_length * math.cos(tilt))

            return tcp_x, tcp_y, tcp_z
        except Exception as e:
            self.get_logger().error(f"FK calculation failed: {e}")
            return None

    def _broadcast_tf(self, x, y, z, tilt, spin, child_frame):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "delta_robot/world_link"
        t.child_frame_id = child_frame

        # Applying -90deg CW rotation to align User Frame with Robot/SDF Frame
        t.transform.translation.x = float(y)
        t.transform.translation.y = -float(x)
        t.transform.translation.z = float(z)

        # Convert tilt/spin to quaternion
        # Rotate orientation by -90deg CW (-pi/2 yaw)
        target_yaw = spin - (math.pi / 2.0)
        cy = math.cos(target_yaw * 0.5)
        sy = math.sin(target_yaw * 0.5)
        cp = math.cos(0.0)
        sp = math.sin(0.0)
        cr = math.cos(tilt * 0.5)
        sr = math.sin(tilt * 0.5)

        t.transform.rotation.w = cr * cp * cy + sr * sp * sy
        t.transform.rotation.x = sr * cp * cy - cr * sp * sy
        t.transform.rotation.y = cr * sp * cy + sr * cp * sy
        t.transform.rotation.z = cr * cp * sy - sr * sp * cy

        self.tf_broadcaster.sendTransform(t)

    def joint_state_callback(self, msg):
        """Processes actual joint state feedback."""
        # Find indices for joints
        try:
            names = msg.name
            t1 = msg.position[names.index("motor_joint_1")]
            t2 = msg.position[names.index("motor_joint_2")]
            t3 = msg.position[names.index("motor_joint_3")]
            tilt = msg.position[names.index("differential_T_joint")]
            spin = msg.position[names.index("differential_EE_joint")]

            res = self._get_tcp_from_joints(t1, t2, t3, tilt, spin)
            if res:
                self._broadcast_tf(*res, tilt, spin, "delta_robot/actual_fk_end_effector_pin")
        except (ValueError, IndexError):
            pass

    def command_callback(self, msg):
        """Processes joint commands (Calculated FK)."""
        if not msg.points:
            return
        
        point = msg.points[0]
        try:
            # Assuming standard 7-joint command order from 5DOF controller
            t1, t2, t3 = point.positions[0:3]
            tilt = point.positions[5]
            spin = point.positions[6]

            res = self._get_tcp_from_joints(t1, t2, t3, tilt, spin)
            if res:
                self._broadcast_tf(*res, tilt, spin, "delta_robot/calculated_fk_end_effector_pin")
        except (ValueError, IndexError):
            pass

def main(args=None):
    rclpy.init(args=args)
    node = JointStateFKBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
