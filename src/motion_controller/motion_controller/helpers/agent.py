import rclpy
from geometry_msgs.msg import PoseStamped
import numpy as np
from scipy.spatial.transform import Rotation
import tf2_ros
from tf2_geometry_msgs import do_transform_pose_stamped


class Agent():
    def __init__(self, node, node_name, tf_buffer: tf2_ros.Buffer, use_ground_truth=False):
        self.node_name = node_name
        self.use_ground_truth = use_ground_truth

        if self.use_ground_truth:
            self.agent_pose_sub = node.create_subscription(
                PoseStamped, f'{node_name}/ground_truth_pose', self.received_agent_pose, 1)
        else:
            self.agent_pose_sub = node.create_subscription(
                PoseStamped, f'{node_name}/camera_pose', self.received_agent_pose, 1)

        self.position: tuple[float, float] = (0, 0)
        self.rotation = 0
        self.tf_buffer = tf_buffer

    def received_agent_pose(self, msg: PoseStamped):
        if self.use_ground_truth:
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
