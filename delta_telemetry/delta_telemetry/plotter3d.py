#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import tf2_ros
from tf2_ros import TransformException


class DeltaEEPlotter(Node):
    def __init__(self):
        super().__init__("delta_ee_plotter")

        self.declare_parameter("marker_frame", "delta_robot/world_link")
        self.declare_parameter("publish_rate_hz", 60.0)
        self.declare_parameter("max_points", 1000)
        self.declare_parameter("sim_child_frame", "delta_robot/end_effector_pin")
        self.declare_parameter("commanded_child_frame", "delta_robot/commanded_end_effector_pin")
        self.declare_parameter("calculated_fk_child_frame", "delta_robot/calculated_fk_end_effector_pin")
        self.declare_parameter("actual_fk_child_frame", "delta_robot/actual_fk_end_effector_pin")
        self.declare_parameter("ee_child_frame", "ee_link")

        self.marker_frame = self.get_parameter("marker_frame").get_parameter_value().string_value
        self.publish_rate_hz = max(
            1.0,
            self.get_parameter("publish_rate_hz").get_parameter_value().double_value,
        )
        self.max_points = max(10, self.get_parameter("max_points").get_parameter_value().integer_value)
        sim_child_frame = self.get_parameter("sim_child_frame").get_parameter_value().string_value
        commanded_child_frame = self.get_parameter("commanded_child_frame").get_parameter_value().string_value
        calculated_fk_child_frame = self.get_parameter("calculated_fk_child_frame").get_parameter_value().string_value
        actual_fk_child_frame = self.get_parameter("actual_fk_child_frame").get_parameter_value().string_value
        ee_child_frame = self.get_parameter("ee_child_frame").get_parameter_value().string_value

        # TF Buffer and Listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Publisher for 3D trajectory lines
        marker_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.marker_pub = self.create_publisher(Marker, "/delta_robot/ee_path", marker_qos)

        self.sim_marker = self._create_line_marker(
            marker_id=0,
            namespace="ee_trajectory_sim",
            color=(0.0, 1.0, 0.0),
        )
        self.commanded_marker = self._create_line_marker(
            marker_id=1,
            namespace="ee_trajectory_commanded",
            color=(1.0, 0.45, 0.0),
        )
        self.calculated_fk_marker = self._create_line_marker(
            marker_id=2,
            namespace="ee_trajectory_calculated_fk",
            color=(0.1, 0.7, 1.0),
        )
        self.actual_fk_marker = self._create_line_marker(
            marker_id=3,
            namespace="ee_trajectory_actual_fk",
            color=(1.0, 0.2, 0.4),
        )
        self.ee_marker = self._create_line_marker(
            marker_id=4,
            namespace="ee_trajectory_sensor",
            color=(0.9, 0.9, 0.1),
        )

        self.tf_traces = [
            {
                "marker": self.sim_marker,
                "parent": self.marker_frame,
                "child": sim_child_frame,
                "error_attr": "_last_sim_transform_error",
            },
            {
                "marker": self.commanded_marker,
                "parent": self.marker_frame,
                "child": commanded_child_frame,
                "error_attr": "_last_commanded_transform_error",
            },
            {
                "marker": self.calculated_fk_marker,
                "parent": self.marker_frame,
                "child": calculated_fk_child_frame,
                "error_attr": "_last_calculated_fk_transform_error",
            },
            {
                "marker": self.actual_fk_marker,
                "parent": self.marker_frame,
                "child": actual_fk_child_frame,
                "error_attr": "_last_actual_fk_transform_error",
            },
            {
                "marker": self.ee_marker,
                "parent": self.marker_frame,
                "child": ee_child_frame,
                "error_attr": "_last_ee_transform_error",
            },
        ]

        self._last_sim_transform_error = ""
        self._last_commanded_transform_error = ""
        self._last_calculated_fk_transform_error = ""
        self._last_actual_fk_transform_error = ""
        self._last_ee_transform_error = ""

        self.get_logger().info(
            f"EE plotter configured with marker_frame={self.marker_frame}, "
            f"publish_rate_hz={self.publish_rate_hz:.1f}, max_points={self.max_points}"
        )

        # Timer to sample and publish path markers at a bounded rate.
        self.timer = self.create_timer(1.0 / self.publish_rate_hz, self.timer_callback)

    def _create_line_marker(self, marker_id: int, namespace: str, color: tuple[float, float, float]) -> Marker:
        marker = Marker()
        marker.header.frame_id = self.marker_frame
        marker.ns = namespace
        marker.id = marker_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.0005
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = 1.0
        return marker

    def _update_marker_from_tf(self, marker: Marker, parent_frame: str, child_frame: str, error_attr: str):
        try:
            trans = self.tf_buffer.lookup_transform(parent_frame, child_frame, rclpy.time.Time())

            p = Point()
            p.x = trans.transform.translation.x
            p.y = trans.transform.translation.y
            p.z = trans.transform.translation.z

            marker.points.append(p)
            if len(marker.points) > self.max_points:
                marker.points.pop(0)
            setattr(self, error_attr, "")

        except TransformException as ex:
            error_msg = str(ex)
            if error_msg != getattr(self, error_attr):
                self.get_logger().info(f"Waiting for valid transform {parent_frame} -> {child_frame}: {error_msg}")
                setattr(self, error_attr, error_msg)

    def timer_callback(self):
        for trace in self.tf_traces:
            self._update_marker_from_tf(
                trace["marker"],
                trace["parent"],
                trace["child"],
                trace["error_attr"],
            )

        # Zero timestamp asks RViz to use the latest transform and prevents
        # TF filter backlog when TF and marker updates are not perfectly synchronized.
        for trace in self.tf_traces:
            marker = trace["marker"]
            marker.header.stamp.sec = 0
            marker.header.stamp.nanosec = 0
            self.marker_pub.publish(marker)


def main():
    rclpy.init()
    node = DeltaEEPlotter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
