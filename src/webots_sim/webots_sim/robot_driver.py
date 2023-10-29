import rclpy
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu


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

        self.prev_velocity = None
        self.prev_timestamp = None
        self.imu_publisher = self.node.create_publisher(
            Imu, f'{self.robot.getName()}/imu', 10)

    def publish_imu_data(self):
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

        if (self.prev_velocity is not None and self.prev_timestamp is not None):
            imu_msg = Imu()
            timestamp = self.robot.getTime()

            imu_msg.header.stamp = self.node.get_clock().now().to_msg()

            imu_msg.angular_velocity.x = robot_rotation[0]
            imu_msg.angular_velocity.y = robot_rotation[1]
            imu_msg.angular_velocity.z = robot_rotation[2]

            robot_accel = (
                (robot_velocity - self.prev_velocity) /
                (timestamp - self.prev_timestamp)
            ) if timestamp != self.prev_timestamp else [0.0, 0.0, -9.8]
            imu_msg.linear_acceleration.x = robot_accel[0]
            imu_msg.linear_acceleration.y = robot_accel[1]
            imu_msg.linear_acceleration.z = robot_accel[2] - 9.8

            self.imu_publisher.publish(imu_msg)

        self.prev_velocity = robot_velocity
        self.prev_timestamp = self.robot.getTime()

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

        self.publish_imu_data()
