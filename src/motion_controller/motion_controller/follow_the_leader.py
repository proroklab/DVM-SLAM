import time
from typing import Tuple
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import numpy as np
import time
from scipy.spatial.transform import Rotation
import tf2_ros
from .helpers.agent import Agent

TIME_STEP = 1/20
LINEAR_GAIN = 1.0
ANGULAR_GAIN = -1.0


class FollowTheLeader(Node):
    def __init__(self, agent_names):
        super().__init__('follow_the_leader')

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.agent_names = agent_names

        self.declare_parameter('agentId', 1)
        self.node_name = f"robot{self.get_parameter('agentId').value}"

        self.declare_parameter('cmdVelTopic', f'{self.node_name}/cmd_vel')
        self.cmd_vel_topic = self.get_parameter('cmdVelTopic').value

        self.declare_parameter('leaderIndex', 0)
        self.leader_index = self.get_parameter('leaderIndex').value

        self.declare_parameter('positionOffsetX', 0.0)
        self.declare_parameter('positionOffsetY', 0.0)
        self.position_offset = (self.get_parameter('positionOffsetX').value,
                                self.get_parameter('positionOffsetY').value)

        self.declare_parameter('rotationOffset', np.pi/2)
        self.rotation_offset = self.get_parameter('rotationOffset').value

        self.agents = []
        for agent_name in self.agent_names:
            self.agents.append(
                Agent(self, agent_name, self.tf_buffer, use_ground_truth=False))

        self.this_agent = self.agents[self.agent_names.index(self.node_name)]

        self.cmd_vel_pub = self.create_publisher(
            Twist, self.cmd_vel_topic, 10)

    def follow_the_leader(self):
        if (any([agent.position is None for agent in self.agents])):
            return

        leader_position = self.agents[self.leader_index].position
        leader_rotation = self.agents[self.leader_index].rotation

        rotated_position_offset = Rotation.from_euler(
            'zyx', [leader_rotation, 0, 0]).as_matrix() @ np.array([self.position_offset[0], self.position_offset[1], 0])
        target_position = (leader_position[0] + rotated_position_offset[0],
                           leader_position[1] + rotated_position_offset[1])
        target_rotation = leader_rotation + self.rotation_offset

        our_position = self.this_agent.position
        our_rotation = self.this_agent.rotation

        print(f"Leader position: {leader_position}")
        print(f"Leader rotation: {leader_rotation}")
        print(f"Our position: {our_position}")
        print(f"Our rotation: {our_rotation}")

        linear_velocity = (
            (target_position[0] - our_position[0]) * LINEAR_GAIN, (target_position[1] - our_position[1]) * LINEAR_GAIN)
        angular_velocity = target_rotation - our_rotation

        if angular_velocity > np.pi:
            angular_velocity -= 2 * np.pi
        elif angular_velocity < -np.pi:
            angular_velocity += 2 * np.pi

        angular_velocity *= ANGULAR_GAIN

        # change velcoty from world to robot coordinates
        inv_rotation_matrix = Rotation.from_euler(
            'zyx', [our_rotation, 0, 0]).inv().as_matrix()
        linear_velocity = inv_rotation_matrix @ np.array(
            [linear_velocity[0], linear_velocity[1], 0])

        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = linear_velocity[0]
        cmd_vel_msg.linear.y = linear_velocity[1]
        cmd_vel_msg.angular.z = angular_velocity

        print(f"Linear velocity: {linear_velocity}")
        print(f"Angular velocity: {angular_velocity}")

        self.cmd_vel_pub.publish(cmd_vel_msg)


def main(args=None):
    rclpy.init(args=args)

    agent_names = ["robot1", "robot2"]

    follow_the_leader = FollowTheLeader(agent_names)
    last_step_time = 0

    try:
        while rclpy.ok():
            if time.time() - last_step_time > TIME_STEP:
                last_step_time = time.time()
                follow_the_leader.follow_the_leader()

            rclpy.spin_once(follow_the_leader)

    except KeyboardInterrupt:
        pass
    finally:
        follow_the_leader.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
