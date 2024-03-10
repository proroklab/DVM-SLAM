import rclpy
import numpy as np
from geometry_msgs.msg import Twist, PoseStamped
from scipy.spatial.transform import Rotation


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

        self.pose_publisher = self.node.create_publisher(
            PoseStamped, f'{self.robot.getName()}/ground_truth_pose', 10)

    def publish_pose(self):
        pose_msg = PoseStamped()

        pose_msg.header.stamp = self.node.get_clock().now().to_msg()
        pose_msg.header.frame_id = "world"

        pose_msg.pose.position.x = self.robot_node.getPosition()[0]
        pose_msg.pose.position.y = self.robot_node.getPosition()[1]
        pose_msg.pose.position.z = self.robot_node.getPosition()[2]

        rotation_matrix = np.array(
            self.robot_node.getOrientation()).reshape((3, 3))
        r = Rotation.from_matrix(rotation_matrix)
        quaternion = r.as_quat()

        pose_msg.pose.orientation.x = quaternion[0]
        pose_msg.pose.orientation.y = quaternion[1]
        pose_msg.pose.orientation.z = quaternion[2]
        pose_msg.pose.orientation.w = quaternion[3]

        self.pose_publisher.publish(pose_msg)

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

        self.publish_pose()
