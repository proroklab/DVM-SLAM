import rclpy
import numpy as np
from geometry_msgs.msg import Twist


class RobotDriver:
    def init(self, webots_node, properties):
        self.robot = webots_node.robot
        self.robot_node = webots_node.robot.getSelf()

        self.camera = self.robot.getDevice("camera")
        self.camera.enable(1)

        self.target_twist = Twist()

        rclpy.init(args=None)
        self.node = rclpy.create_node(f'{self.robot.getName()}_driver')
        self.node.create_subscription(
            Twist, f'{self.robot.getName()}/cmd_vel', self.cmd_vel_callback, 1)

    def cmd_vel_callback(self, twist):
        self.target_twist = twist

    def step(self):
        rclpy.spin_once(self.node, timeout_sec=0)

        rotation_matrix = np.array(
            self.robot_node.getOrientation()).reshape((3, 3))

        # create rotation matrix for only z axis
        theta = np.arctan2(rotation_matrix[1, 0], rotation_matrix[0, 0])
        rotation_z_matrix = np.array([[np.cos(theta), -np.sin(theta), 0],
                                      [np.sin(theta), np.cos(theta), 0],
                                      [0, 0, 1]])

        robot_velocity = np.array([
            self.target_twist.linear.x,
            self.target_twist.linear.y,
            self.target_twist.linear.z
        ])
        robot_rotation = np.array([
            self.target_twist.angular.x,
            self.target_twist.angular.y,
            self.target_twist.angular.z
        ])

        world_velocity = np.dot(rotation_matrix, robot_velocity)
        # only rotate around z axis
        world_rotation = np.dot(rotation_z_matrix, robot_rotation)

        self.robot_node.setVelocity(
            world_velocity.tolist() + world_rotation.tolist())
