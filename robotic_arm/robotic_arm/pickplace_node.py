import rclpy
from rclpy.node import Node

import csv
import os

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Float64MultiArray
from builtin_interfaces.msg import Duration
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy


class TrajectoryPubNode(Node):
    def __init__(self):
        super().__init__('pickplace_loop_node')

        # QoS
        jqos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE
        )

        gqos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )

        # Publishers
        self.arm_pub = self.create_publisher(
            JointTrajectory,
            '/arm_controller/joint_trajectory',
            jqos
        )

        self.gripper_pub = self.create_publisher(
            Float64MultiArray,
            '/gripper_controller/commands',
            gqos
        )

        # ---- ROBOT STATE ----
        self.robot_state = 'home'        # home | holding
        self.object_pose = 'left'        # left | front
        self.current_task = 'pick'       # pick | place
        self.motion_step = 0

        # Joint reference
        self.alpha_left = -1.57
        self.alpha_front = 0.0

        self.alpha = self.alpha_left
        self.gripper_open = False

        # Timer (non-blocking)
        self.create_timer(3, self.control_loop)
        self.get_logger().info('Closed-loop Pick & Place Node Started')

    # ---------------- PUBLISHERS ----------------

    def publish_traj(self, a, b, c, d):
        traj = JointTrajectory()
        traj.joint_names = [
            'Revolute_1',
            'Revolute_2',
            'Revolute_3',
            'Revolute_4'
        ]

        point = JointTrajectoryPoint()
        point.positions = [a, b, c, d]
        point.time_from_start = Duration(sec=2)

        traj.points.append(point)
        self.arm_pub.publish(traj)

    def publish_gripper(self, open_grip):
        msg = Float64MultiArray()
        msg.data = [0.06] if open_grip else [0.04]
        self.gripper_pub.publish(msg)
        self.gripper_open = open_grip

    # ---------------- CONTROL LOOP ----------------

    def control_loop(self):
        if not os.path.exists('/home/zaid-khan/pickplace.csv'):
            return
        
        with open("/home/zaid-khan/pickplace.csv") as f:
            reader = csv.reader(f)
            row = next(reader, None)

        command = row[0]

        if command is None:
            return

        elif command == 'pick':
            self.get_logger().info('Picking Object !')
            self.pick_sequence()

        elif command == 'place':
            self.get_logger().info('Placing Object !')
            self.place_sequence()

    # ---------------- PICK ----------------

    def pick_sequence(self):

        if self.robot_state == 'holding':
            self.current_task = 'place'
            self.motion_step = 0
            return

        # Decide alpha based on object pose
        self.alpha = self.alpha_left if self.object_pose == 'left' else self.alpha_front

        if self.motion_step == 0:
            self.publish_traj(self.alpha, 0.0, 0.0, 0.0)
            self.motion_step += 1

        elif self.motion_step == 1:
            self.publish_traj(self.alpha, -0.4, 0.4, 1.2)
            self.motion_step += 1

        elif self.motion_step == 2:
            self.publish_gripper(False)
            self.motion_step += 1

        elif self.motion_step == 3:
            self.publish_traj(self.alpha, 0.4, 0.6, 0.5)
            self.robot_state = 'holding'
            self.motion_step = 0
            self.current_task = 'place'
            self.get_logger().info('Object picked')

    # ---------------- PLACE ----------------

    def place_sequence(self):

        if self.robot_state != 'holding':
            self.current_task = 'pick'
            self.motion_step = 0
            return

        # Opposite placement logic
        self.alpha = self.alpha_front if self.object_pose == 'left' else self.alpha_left

        if self.motion_step == 0:
            self.publish_traj(self.alpha, 0.4, 0.6, 0.5)
            self.motion_step += 1

        elif self.motion_step == 1:
            self.publish_traj(self.alpha, -0.4, 0.4, 1.2)
            self.motion_step += 1

        elif self.motion_step == 2:
            self.publish_gripper(True)
            self.motion_step += 1

        elif self.motion_step == 3:
            self.publish_traj(self.alpha, 0.0, 0.0, 0.0)
            self.robot_state = 'home'

            # Update object pose
            self.object_pose = 'front' if self.object_pose == 'left' else 'left'

            self.motion_step = 0
            self.current_task = 'pick'
            self.get_logger().info('Object placed')


def main():
    rclpy.init()
    node = TrajectoryPubNode()
    rclpy.spin(node)
    rclpy.shutdown()