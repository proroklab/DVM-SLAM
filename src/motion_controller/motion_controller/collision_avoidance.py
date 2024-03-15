import time
from .helpers.robot_types import RobotTypes
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from interactive_markers.menu_handler import MenuHandler
from visualization_msgs.msg import Marker
import numpy as np
import time
from scipy.spatial.transform import Rotation
import tf2_ros
from .helpers.agent import Agent
from .helpers.nmpc_collision_avoidance import Nmpc
from .helpers.interactive_marker_wrapper import InteractiveMarkerWrapper

TIME_STEP = 1/20
AGENT_RADIUS = 0.25
LINEAR_GAIN = 5.0
ANGULAR_GAIN = 5.0
MAX_LINEAR_SPEED = 5.0
MAX_ANGULAR_SPEED = 5.0
ROBOT_TYPE = RobotTypes.ROBOMASTER


class StaticObstacle:
    def __init__(self, x1, y1, x2, y2, marker_server, menu_handler, name, node):
        self.corner1 = (x1, y1)
        self.corner2 = (x2, y2)

        self.corner1_marker = InteractiveMarkerWrapper(
            f"{name}_corner1", self.corner1, marker_server, menu_handler)
        self.corner2_marker = InteractiveMarkerWrapper(
            f"{name}_corner2", self.corner2, marker_server, menu_handler)

        self.obstacle_rect_pub = node.create_publisher(
            Marker, f'{node.node_name}/{name}', 10)

    def get_corners(self):
        self.publish_marker()

        self.corner1 = self.corner1_marker.position
        self.corner2 = self.corner2_marker.position

        return (self.corner1[0], self.corner1[1], self.corner2[0], self.corner2[1])

    def publish_marker(self):
        marker = Marker()
        marker.header.frame_id = 'world'
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.1  # Line width

        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        # Define the points of the rectangle
        p1 = Point()
        p1.x = float(self.corner1[0])
        p1.y = float(self.corner1[1])
        p1.z = 0.0

        p2 = Point()
        p2.x = float(self.corner2[0])
        p2.y = float(self.corner1[1])
        p2.z = 0.0

        p3 = Point()
        p3.x = float(self.corner2[0])
        p3.y = float(self.corner2[1])
        p3.z = 0.0

        p4 = Point()
        p4.x = float(self.corner1[0])
        p4.y = float(self.corner2[1])
        p4.z = 0.0

        marker.points.append(p1)
        marker.points.append(p2)
        marker.points.append(p3)
        marker.points.append(p4)
        marker.points.append(p1)

        self.obstacle_rect_pub.publish(marker)


class CollisionAvoidance(Node):
    def __init__(self):
        super().__init__('collision_avoidance')

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.nmpc = Nmpc(AGENT_RADIUS, MAX_LINEAR_SPEED,
                         0., TIME_STEP, TIME_STEP*3, 4)

        self.agent_names = ["robot1", "robot2"]

        self.declare_parameter('agentId', 1)
        self.node_name = f"robot{self.get_parameter('agentId').value}"

        self.declare_parameter('cmdVelTopic', f'{self.node_name}/cmd_vel')
        self.cmd_vel_topic = self.get_parameter('cmdVelTopic').value

        self.marker_server = InteractiveMarkerServer(
            self, f"{self.node_name}_marker_server")

        self.agent_index = self.agent_names.index(self.node_name)

        self.agents: list = []
        for agent_name in self.agent_names:
            self.agents.append(Agent(self, agent_name, self.cmd_vel_topic, self.tf_buffer,
                               ROBOT_TYPE, LINEAR_GAIN, ANGULAR_GAIN, MAX_LINEAR_SPEED, MAX_ANGULAR_SPEED))

        self.this_agent: Agent = self.agents[self.agent_index]

        self.menu_handler = MenuHandler()

        self.goal_marker = InteractiveMarkerWrapper(
            f"{self.node_name}_goal_marker", (0, 0), self.marker_server, self.menu_handler)

        self.static_obstacles = [StaticObstacle(
            5+x, 5+x, 6+x, 6+x, self.marker_server, self.menu_handler, f"obstacle{x}", self) for x in range(2)]

    def avoid_collision(self):
        if (any([agent.position is None for agent in self.agents])):
            return

        self.nmpc.set_goal(self.goal_marker.position)
        # print("goal", self.goal_marker.position)

        obstacles = np.array(
            [agent.position for agent in self.agents if agent != self.agents[self.agent_index]])
        static_obstacles = np.array(
            [obstacle.get_corners() for obstacle in self.static_obstacles])

        self.nmpc.set_static_obstacles(static_obstacles)

        velocity = self.nmpc.step(
            self.this_agent.position, obstacles)

        rotation = self.this_agent.rotation
        inv_rotation_matrix = Rotation.from_euler(
            'zyx', [rotation, 0, 0]).inv().as_matrix()
        velocity = inv_rotation_matrix @ np.array(
            [velocity[0], velocity[1], 0.0])
        # print(velocity)

        self.this_agent.set_velocity(velocity, 0.0)


def main(args=None):
    rclpy.init(args=args)
    collision_avoidance = CollisionAvoidance()
    last_step_time = 0

    try:
        while rclpy.ok():
            if time.time() - last_step_time > TIME_STEP:
                last_step_time = time.time()
                collision_avoidance.avoid_collision()

            rclpy.spin_once(collision_avoidance)

    except KeyboardInterrupt:
        pass
    finally:
        collision_avoidance.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
