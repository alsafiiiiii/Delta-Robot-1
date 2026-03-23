import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

def main():
    rclpy.init()
    node = Node('send_joint_angles')
    pub = node.create_publisher(JointTrajectory, '/delta/joint_commands', 10)

    msg = JointTrajectory()
    msg.joint_names = ['jbf1', 'jbf2', 'jbf3', 'Bevelj1', 'Bevelj2']
    point = JointTrajectoryPoint()
    # Set your desired angles here (in degrees or radians as expected by your firmware)
    point.positions = [90.0, 90.0, 90.0, 90.0, 90.0]
    point.time_from_start.sec = 0
    point.time_from_start.nanosec = 10000000
    msg.points.append(point)

    pub.publish(msg)
    print("Joint angles sent.")
    rclpy.shutdown()

if __name__ == '__main__':
    main()