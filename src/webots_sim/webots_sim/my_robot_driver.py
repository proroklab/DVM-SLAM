import rclpy
import numpy as np
from geometry_msgs.msg import Twist

HALF_DISTANCE_BETWEEN_WHEELS = 0.045
WHEEL_RADIUS = 0.025

class MyRobotDriver:
    def init(self, webots_node, properties):
        self.robot = webots_node.robot
        self.robot_node = webots_node.robot.getSelf()

        self.camera = self.robot.getDevice("camera")
        self.camera.enable(1) 

        self.target_twist = Twist()

        rclpy.init(args=None)
        self.node = rclpy.create_node('my_robot_driver')
        self.node.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 1)

    def cmd_vel_callback(self, twist):
        self.target_twist = twist

    def step(self):
        rclpy.spin_once(self.node, timeout_sec=0)

        world_to_robot_rotation_matrix = np.array(self.robot_node.getOrientation()).reshape((3,3))
        robot_to_world_rotation_matrix = np.transpose(world_to_robot_rotation_matrix)
        
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

        world_velocity = np.dot(world_to_robot_rotation_matrix, robot_velocity)
        world_rotation = np.dot(world_to_robot_rotation_matrix, robot_rotation)

        self.robot_node.setVelocity(world_velocity.tolist() + world_rotation.tolist())