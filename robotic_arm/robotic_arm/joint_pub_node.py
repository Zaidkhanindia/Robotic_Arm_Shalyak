import rclpy
from rclpy.node import Node

import csv
import math
import os

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Float64MultiArray
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

        gqos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )

        self.pub = self.create_publisher(
            JointTrajectory,
            '/arm_controller/joint_trajectory',
            qos
        )

        self.gripper_pub = self.create_publisher(
            Float64MultiArray,
            '/gripper_controller/commands',
            gqos
        )

        self.joint_state = {
            'alpha' : 0.0,
            'beta'  : 0.0,
            'gamma' : 0.0,
            'delta' : 0.0
        }

        self.create_timer(3, self.read_and_publish)
        self.get_logger().info('Trajectory publisher started...!')

    def read_and_publish(self):
        if not os.path.exists('/home/zaid-khan/command.csv'):
            return
        
        with open("/home/zaid-khan/command.csv") as f:
            reader = csv.reader(f)
            row = next(reader, None)

        if row is None:
            return
        
        elif row[0] and row[1] is not None:
            joint = row[0]
            angle = float(row[1])
            rad = math.radians(angle)

            if joint in self.joint_state:
                self.joint_state[joint] = rad

            self.publish_cmd(
                self.joint_state['alpha'],
                self.joint_state['beta'],
                self.joint_state['gamma'],
                self.joint_state['delta']
            )

        elif row[2] is not None:
            if row[2] == 'open':
                self.publish_gripper(True)
            else:
                self.publish_gripper(False)


    def publish_cmd(self, alpha, beta, gamma, delta):
        traj = JointTrajectory()
        traj.joint_names = ['Revolute_1', 'Revolute_2', 'Revolute_3', 'Revolute_4']

        point = JointTrajectoryPoint()
        point.positions = [alpha, beta, gamma, delta]
        point.time_from_start = Duration(sec=3)

        traj.points.append(point)
        self.pub.publish(traj)

        self.get_logger().info('Trajectory published!')
        self.get_logger().info(f'Alpha: {self.joint_state['alpha']:.2f}')
        self.get_logger().info(f'Beta: {self.joint_state['beta']:.2f}')
        self.get_logger().info(f'Gamma: {self.joint_state['gamma']:.2f}')
        self.get_logger().info(f'Gamma: {self.joint_state['delta']:.2f}')
        self.get_logger().info('-----------------------------------')

    def publish_gripper(self, open_grip):
        msg = Float64MultiArray()
        msg.data = [0.06] if open_grip else [0.00]
        gstate = 'Open' if open_grip else 'Close'
        self.gripper_pub.publish(msg)
        self.get_logger().info(f'Gripper {gstate}!')




def main():
    rclpy.init()
    node = TrajectoryPubNode()
    rclpy.spin(node)
    rclpy.shutdown()
