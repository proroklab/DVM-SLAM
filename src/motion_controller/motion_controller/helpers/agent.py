import rclpy
from geometry_msgs.msg import PoseStamped
import numpy as np
from scipy.spatial.transform import Rotation
import tf2_ros
from tf2_geometry_msgs import do_transform_pose_stamped
from .robot_types import RobotTypes
from geometry_msgs.msg import Twist


class Agent():
    def __init__(self, node, node_name, tf_buffer: tf2_ros.Buffer, robot_type: RobotTypes, linear_gain, angular_gain, max_linear_speed, max_angular_speed):
        self.node_name = node_name
        self.robot_type = robot_type

        self.linear_gain = linear_gain
        self.angular_gain = angular_gain
        self.max_linear_speed = max_linear_speed
        self.max_angular_speed = max_angular_speed

        if self.robot_type == RobotTypes.SIM_GROUND_TRUTH:
            self.agent_pose_sub = node.create_subscription(
                PoseStamped, f'{node_name}/ground_truth_pose', self.received_agent_pose, 1)
        else:
            self.agent_pose_sub = node.create_subscription(
                PoseStamped, f'{node_name}/camera_pose', self.received_agent_pose, 1)

        self.position: tuple[float, float] = (0, 0)
        self.rotation = 0
        self.tf_buffer = tf_buffer

    def received_agent_pose(self, msg: PoseStamped):
        if self.robot_type == RobotTypes.SIM_GROUND_TRUTH:
            self.position = (msg.pose.position.x,
                             msg.pose.position.y)

            rotation_3d = Rotation.from_quat(
                [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]).as_matrix()
            direction_vector = rotation_3d @ np.array([1, 0, 0])
            self.rotation = np.arctan2(
                direction_vector[1], direction_vector[0])

        else:
            try:
                transform = self.tf_buffer.lookup_transform(
                    "world", msg.header.frame_id, rclpy.time.Time())
                msg = do_transform_pose_stamped(msg, transform)
            except Exception as e:
                print(e)
                pass

            self.position = (msg.pose.position.x,
                             msg.pose.position.y)
            # print(self.node_name, self.position)
            rotation_3d = Rotation.from_quat(
                [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]).as_matrix()
            direction_vector = rotation_3d @ np.array([1, 0, 0])
            self.rotation = np.arctan2(
                direction_vector[1], direction_vector[0])

    def set_velocity(self, cmd_vel_pub, linear_velocity, angular_velocity):
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

        cmd_vel_pub.publish(cmd_vel_msg)

    def move_to_position(self, cmd_vel_pub, position, rotation):
        our_position = self.position
        our_rotation = self.rotation

        print(f"Leader position: {position}")
        print(f"Leader rotation: {rotation}")
        print(f"Our position: {our_position}")
        print(f"Our rotation: {our_rotation}")

        linear_velocity = (
            (position[0] - our_position[0]) * self.linear_gain, (position[1] - our_position[1]) * self.linear_gain)
        angular_velocity = rotation - our_rotation

        if angular_velocity > np.pi:
            angular_velocity -= 2 * np.pi
        elif angular_velocity < -np.pi:
            angular_velocity += 2 * np.pi

        angular_velocity *= self.angular_gain

        # change velcoty from world to robot coordinates
        inv_rotation_matrix = Rotation.from_euler(
            'zyx', [our_rotation, 0, 0]).inv().as_matrix()
        linear_velocity = inv_rotation_matrix @ np.array(
            [linear_velocity[0], linear_velocity[1], 0])

        # limit speeds
        if np.linalg.norm(linear_velocity) != 0:
            linear_velocity *= (self.max_linear_speed /
                                np.linalg.norm(linear_velocity))
        angular_velocity = np.clip(
            angular_velocity, -self.max_angular_speed, self.max_angular_speed)

        print(f"Linear velocity: {linear_velocity}")
        print(f"Angular velocity: {angular_velocity}")

        self.set_velocity(cmd_vel_pub, linear_velocity, angular_velocity)
