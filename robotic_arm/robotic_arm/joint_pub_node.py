import rclpy
from rclpy.node import Node

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy


class TrajectoryPubNode(Node):
    def __init__(self):
        super().__init__('joint_pub_node')

        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE
        )

        self.pub = self.create_publisher(
            JointTrajectory,
            '/arm_controller/joint_trajectory',
            qos
        )

        self.create_timer(3, self.publish_cmd(1.57, 0.0, 0.0))

        self.get_logger().info('Trajectory publisher started')

    def publish_cmd(self, alpha, beta, gamma):
        traj = JointTrajectory()
        traj.joint_names = ['Revolute_1', 'Revolute_2', 'Revolute_3']

        point = JointTrajectoryPoint()
        point.positions = [alpha, beta, gamma]
        point.time_from_start = Duration(sec=3)

        traj.points.append(point)
        self.pub.publish(traj)

        self.get_logger().info('Trajectory published')


def main():
    rclpy.init()
    node = TrajectoryPubNode()
    rclpy.spin(node)
    rclpy.shutdown()
