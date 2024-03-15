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
from .helpers.robot_types import RobotTypes

TIME_STEP = 1/20
LINEAR_GAIN = 5.0
ANGULAR_GAIN = 5.0
MAX_LINEAR_SPEED = 5.0
MAX_ANGULAR_SPEED = 5.0
ROBOT_TYPE = RobotTypes.ROBOMASTER


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
                Agent(self, agent_name, self.cmd_vel_topic, self.tf_buffer, ROBOT_TYPE, LINEAR_GAIN, ANGULAR_GAIN, MAX_LINEAR_SPEED, MAX_ANGULAR_SPEED))

        self.this_agent = self.agents[self.agent_names.index(self.node_name)]

    def follow_the_leader(self):
        if (any([agent.position is None for agent in self.agents])):
            return

        leader_position = self.agents[self.leader_index].position
        leader_rotation = self.agents[self.leader_index].rotation

        self.this_agent.move_to_position(leader_position, leader_rotation)


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
