import random
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, Pose, Point, Vector3
from std_msgs.msg import Float32
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from interactive_markers.menu_handler import MenuHandler
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, Marker, InteractiveMarkerFeedback
from typing import Tuple
import numpy as np
from scipy.optimize import minimize, Bounds
import time
from scipy.spatial.transform import Rotation
import tf2_ros
from tf2_ros import TransformListener
from tf2_geometry_msgs import do_transform_pose_stamped, do_transform_vector3

TIME_STEP = 1/20
AGENT_RADIUS = 0.25
AGENT_MAX_SPEED = 5.0


class Nmpc():
    """
    Collision avoidance using Nonlinear Model-Predictive Control

    Adapted from Ashwin Bose 
    https://github.com/atb033/multi_agent_path_planning/blob/master/decentralized/nmpc/nmpc.py
    """

    def __init__(self, robot_radius, vmax, vmin, timestep=0.1, nmpc_timestep=0.3,  horizon_length=int(4), static_obstacles=[]):
        self.timestep = timestep
        self.robot_radius = robot_radius
        self.vmax = vmax
        self.vmin = vmin

        # collision cost parameters
        # https://www.desmos.com/calculator/lu9hv6mq36
        self.Qc = 5.
        self.kappa = 4.
        self.static_kappa = 40.

        # nmpc parameters
        self.horizon_length = horizon_length
        self.nmpc_timestep = nmpc_timestep
        self.upper_bound = [(1/np.sqrt(2)) * self.vmax] * \
            self.horizon_length * 2
        self.lower_bound = [-(1/np.sqrt(2)) * self.vmax] * \
            self.horizon_length * 2
        self.goal = (0, 0)

        # num_timesteps, num_obstacles, Tuple[float, float] array
        self.obstacle_position_history = None

        # rectangle corners (x1, y1, x2, y2)
        self.static_obstacles = static_obstacles

    def set_static_obstacles(self, static_obstacles):
        self.static_obstacles = static_obstacles

    def set_goal(self, goal: Tuple[float, float]):
        self.goal = np.array(goal)

    def step(self, position, obstacle_positions: np.ndarray[Tuple[float, float]]) -> Tuple[float, float]:
        robot_state = np.array(position)

        obstacle_predictions = self.predict_obstacle_positions(
            obstacle_positions)
        xref = self.compute_xref(
            robot_state, self.goal, self.horizon_length, self.nmpc_timestep)
        # compute velocity using nmpc
        vel, velocity_profile = self.compute_velocity(
            robot_state, obstacle_predictions, xref)
        robot_state = self.update_state(robot_state, vel, self.timestep)

        return (vel[0], vel[1])

    def compute_velocity(self, robot_state, obstacle_predictions, xref):
        """
        Computes control velocity of the copter
        """
        # u0 = np.array([0] * 2 * self.horizon_length)
        u0 = np.random.rand(2*self.horizon_length)
        def cost_fn(u): return self.total_cost(
            u, robot_state, obstacle_predictions, xref)

        bounds = Bounds(self.lower_bound, self.upper_bound)

        res = minimize(cost_fn, u0, method='SLSQP', bounds=bounds)
        velocity = res.x[:2]
        return velocity, res.x

    def compute_xref(self, start, goal, number_of_steps, timestep):
        dir_vec = (goal - start)
        norm = np.linalg.norm(dir_vec)
        if norm < 0.1:
            new_goal = start
        else:
            dir_vec = dir_vec / norm
            new_goal = start + dir_vec * self.vmax * timestep * number_of_steps
        return np.linspace(start, new_goal, number_of_steps).reshape((2*number_of_steps))

    def total_cost(self, u, robot_state, obstacle_predictions, xref):
        x_robot = self.update_state(robot_state, u, self.nmpc_timestep)
        c1 = self.tracking_cost(x_robot, xref)
        c2 = self.total_collision_cost(x_robot, obstacle_predictions)
        total = c1 + c2
        return total

    def tracking_cost(self, x, xref):
        return np.linalg.norm(x-xref)

    def total_collision_cost(self, robot, obstacles):
        total_cost = 0
        for i in range(self.horizon_length):
            for j in range(len(obstacles)):
                obstacle = obstacles[j]
                rob = robot[2 * i: 2 * i + 2]
                obs = obstacle[2 * i: 2 * i + 2]
                total_cost += self.collision_cost(rob, obs)
            for static_obstacle in self.static_obstacles:
                rob = robot[2 * i: 2 * i + 2]
                distance = self.distance_point_to_rectangle(
                    rob, static_obstacle)
                cost = self.Qc / \
                    (1 + np.exp(self.static_kappa * (distance - self.robot_radius)))
                total_cost += cost
        return total_cost

    def collision_cost(self, x0, x1):
        """
        Cost of collision between two robot_state
        """
        d = np.linalg.norm(x0 - x1)
        cost = self.Qc / (1 + np.exp(self.kappa * (d - 2*self.robot_radius)))
        return cost

    def distance_point_to_rectangle(self, point, rectangle):
        # Extracting coordinates
        x, y = point
        x1, y1, x2, y2 = rectangle

        # Finding rectangle boundaries
        min_x = min(x1, x2)
        max_x = max(x1, x2)
        min_y = min(y1, y2)
        max_y = max(y1, y2)

        # Finding distance to the closest edge
        if x < min_x:
            dx = min_x - x
        elif x > max_x:
            dx = x - max_x
        else:
            dx = 0

        if y < min_y:
            dy = min_y - y
        elif y > max_y:
            dy = y - max_y
        else:
            dy = 0

        # Calculating distance
        distance = np.sqrt(dx**2 + dy**2)
        return distance

    def predict_obstacle_positions(self, obstacle_positions: np.ndarray[Tuple[float, float]]):
        obstacle_predictions = []
        for i in range(len(obstacle_positions)):
            if self.obstacle_position_history is not None:
                obstacle_vel = (np.array(obstacle_positions[i]) - np.array(
                    self.obstacle_position_history[-1][i])) / self.timestep
            else:
                obstacle_vel = np.array([0, 0])

            obstacle_position = np.array(obstacle_positions[i])
            u = np.vstack([np.eye(2)] * self.horizon_length) @ obstacle_vel
            obstacle_prediction = self.update_state(
                obstacle_position, u, self.nmpc_timestep)
            obstacle_predictions.append(obstacle_prediction)

        if self.obstacle_position_history is None:
            self.obstacle_position_history = np.array([obstacle_positions])
        else:
            self.obstacle_position_history = np.append(
                self.obstacle_position_history, [obstacle_positions], axis=0)

        return obstacle_predictions

    def update_state(self, x0, u, timestep):
        """
        Computes the states of the system after applying a sequence of control signals u on
        initial state x0
        """
        N = int(len(u) / 2)
        lower_triangular_ones_matrix = np.tril(np.ones((N, N)))
        kron = np.kron(lower_triangular_ones_matrix, np.eye(2))

        new_state = np.vstack([np.eye(2)] * int(N)) @ x0 + kron @ u * timestep

        return new_state


class Agent():
    def __init__(self, node, node_name, tf_buffer: tf2_ros.Buffer):
        self.node_name = node_name
        self.agent_pose_sub = node.create_subscription(
            PoseStamped, f'{node_name}/camera_pose', self.received_agent_pose, 1)
        self.position: tuple[float, float] = None
        self.tf_buffer = tf_buffer

    def received_agent_pose(self, msg: PoseStamped):
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
        self.rotation = -np.arctan2(direction_vector[1], direction_vector[0])


class InteractiveMarkerWrapper():
    def __init__(self, name: str, position: Tuple[float, float], marker_server, menu_handler):
        self.name = name
        self.position = position
        self.marker_server = marker_server
        self.menu_handler = menu_handler

        self.create_marker()

    def create_marker(self):
        interactive_marker = InteractiveMarker()
        interactive_marker.header.frame_id = "world"
        interactive_marker.name = self.name
        interactive_marker.pose.position.x = float(self.position[0])
        interactive_marker.pose.position.y = float(self.position[1])
        interactive_marker.pose.position.z = 0.5
        interactive_marker.scale = 0.5

        control = InteractiveMarkerControl()
        control.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
        control.orientation.w = 1.0
        control.orientation.x = 0.0
        control.orientation.y = 1.0
        control.orientation.z = 0.0
        control.always_visible = True

        marker = Marker()
        marker.type = Marker.SPHERE
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        control.markers.append(marker)
        interactive_marker.controls.append(control)

        self.marker_server.insert(
            interactive_marker, feedback_callback=self.marker_feedback)
        self.menu_handler.apply(self.marker_server, interactive_marker.name)

        self.marker_server.applyChanges()

    def marker_feedback(self, feedback):
        if feedback.event_type == feedback.POSE_UPDATE:
            self.position = (feedback.pose.position.x,
                             feedback.pose.position.y)


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

        self.nmpc = Nmpc(AGENT_RADIUS, AGENT_MAX_SPEED,
                         0., TIME_STEP, TIME_STEP*3, 4)

        self.agent_names = ["robot1", "robot2"]

        self.declare_parameter('agentId', 1)
        self.node_name = f"robot{self.get_parameter('agentId').value}"

        self.marker_server = InteractiveMarkerServer(
            self, f"{self.node_name}_marker_server")

        self.agent_index = self.agent_names.index(self.node_name)

        self.agents: list[Agent] = []
        for agent_name in self.agent_names:
            self.agents.append(Agent(self, agent_name, self.tf_buffer))

        self.cmd_vel_pub = self.create_publisher(
            Twist, f'{self.node_name}/cmd_vel', 10)

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
            self.agents[self.agent_index].position, obstacles)

        rotation = self.agents[self.agent_index].rotation
        rotation_matrix = Rotation.from_euler(
            'zyx', [rotation, 0, 0]).as_matrix()
        velocity = rotation_matrix @ np.array([velocity[0], velocity[1], 0])
        # print(velocity)

        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = velocity[0]
        cmd_vel_msg.linear.y = velocity[1]

        self.cmd_vel_pub.publish(cmd_vel_msg)


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
