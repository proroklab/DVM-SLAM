from geometry_msgs.msg import Twist
from .robot_types import RobotTypes
from scipy.spatial.transform import Rotation
import numpy as np


class Driver():
    def __init__(self, node, robot_type, cmd_vel_topic, linear_gain, angular_gain, max_linear_speed, max_angular_speed):
        self.linear_gain = linear_gain
        self.angular_gain = angular_gain
        self.max_linear_speed = max_linear_speed
        self.max_angular_speed = max_angular_speed

        self.robot_type = robot_type

        self.cmd_vel_pub = node.create_publisher(
            Twist, self.cmd_vel_topic, 10)

    def set_velocity(self, linear_velocity, angular_velocity):
        # limit speeds
        if np.linalg.norm(linear_velocity) != 0:
            linear_velocity *= (self.max_linear_speed /
                                np.linalg.norm(linear_velocity))
        angular_velocity = np.clip(
            angular_velocity, -self.max_angular_speed, self.max_angular_speed)

        print(f"Linear velocity: {linear_velocity}")
        print(f"Angular velocity: {angular_velocity}")

        if self.robot_type == RobotTypes.ROBOMASTER:
            cmd_vel_msg = Twist()
            cmd_vel_msg.linear.x = linear_velocity[0]
            cmd_vel_msg.linear.y = -linear_velocity[1]
            cmd_vel_msg.angular.z = -angular_velocity
        elif self.robot_type == RobotTypes.SIM:
            cmd_vel_msg = Twist()
            cmd_vel_msg.linear.x = linear_velocity[0]
            cmd_vel_msg.linear.y = linear_velocity[1]
            cmd_vel_msg.angular.z = angular_velocity
        elif self.robot_type == RobotTypes.SIM_GROUND_TRUTH:
            cmd_vel_msg = Twist()
            cmd_vel_msg.linear.x = linear_velocity[0]
            cmd_vel_msg.linear.y = linear_velocity[1]
            cmd_vel_msg.angular.z = angular_velocity

        self.cmd_vel_pub.publish(cmd_vel_msg)

    def move_to_position(self, target_position, target_rotation, current_position, current_rotation):
        print(f"target position: {target_position}")
        print(f"target rotation: {target_rotation}")
        print(f"current position: {current_position}")
        print(f"current rotation: {current_rotation}")

        linear_velocity = (
            (target_position[0] - current_position[0]) * self.linear_gain, (target_position[1] - current_position[1]) * self.linear_gain)
        angular_velocity = target_rotation - current_rotation

        if angular_velocity > np.pi:
            angular_velocity -= 2 * np.pi
        elif angular_velocity < -np.pi:
            angular_velocity += 2 * np.pi

        angular_velocity *= self.angular_gain

        # change velcoty from world to robot coordinates
        inv_rotation_matrix = Rotation.from_euler(
            'zyx', [current_rotation, 0, 0]).inv().as_matrix()
        linear_velocity = inv_rotation_matrix @ np.array(
            [linear_velocity[0], linear_velocity[1], 0])

        self.set_velocity(linear_velocity, angular_velocity)
