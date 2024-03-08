import random
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, Pose
from std_msgs.msg import Float32
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from interactive_markers.menu_handler import MenuHandler
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, Marker
from typing import Tuple
import numpy as np
from scipy.optimize import minimize, Bounds
import time

TIME_STEP = 1/5
AGENT_RADIUS = 0.5
AGENT_MAX_SPEED = 8.0


class Nmpc():
    """
    Collision avoidance using Nonlinear Model-Predictive Control

    Adapted from Ashwin Bose 
    https://github.com/atb033/multi_agent_path_planning/blob/master/decentralized/nmpc/nmpc.py
    """

    def __init__(self, robot_radius, vmax, vmin, timestep=0.1, nmpc_timestep=0.3,  horizon_length=int(4)):
        self.timestep = timestep
        self.robot_radius = robot_radius
        self.vmax = vmax
        self.vmin = vmin

        # collision cost parameters
        self.Qc = 50.
        self.kappa = 4.

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

    def set_goal(self, goal: Tuple[float, float]):
        self.goal = np.array(goal)

    def step(self, position, obstacle_positions: np.ndarray[Tuple[float, float]]) -> Tuple[float, float]:
        robot_state = np.array(position)

        obstacle_predictions = self.predict_obstacle_positions(
            obstacle_positions)
        print(obstacle_predictions)
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
        return total_cost

    def collision_cost(self, x0, x1):
        """
        Cost of collision between two robot_state
        """
        d = np.linalg.norm(x0 - x1)
        cost = self.Qc / (1 + np.exp(self.kappa * (d - 2*self.robot_radius)))
        return cost

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
    def __init__(self, node, node_name):
        self.node_name = node_name
        self.agent_pose_sub = node.create_subscription(
            PoseStamped, f'{node_name}/camera_pose', self.received_agent_pose, 1)
        self.position: tuple[float, float] = None

    def received_agent_pose(self, msg: PoseStamped):
        self.position = (msg.pose.position.x, msg.pose.position.z)


class CollisionAvoidance(Node):
    def __init__(self):
        super().__init__('collision_avoidance')

        self.nmpc = Nmpc(AGENT_RADIUS, AGENT_MAX_SPEED, 0., 0.1, 0.3, 4)

        self.agent_names = ["robot1", "robot2"]

        self.declare_parameter('agentId', 1)
        self.node_name = f"robot{self.get_parameter('agentId').value}"

        self.marker_server = InteractiveMarkerServer(
            self, f"{self.node_name}_marker_server")

        self.agent_index = self.agent_names.index(self.node_name)

        self.agents: list[Agent] = []
        for agent_name in self.agent_names:
            self.agents.append(Agent(self, agent_name))

        self.cmd_vel_pub = self.create_publisher(
            Twist, f'{self.node_name}/cmd_vel', 10)

        self.menu_handler = MenuHandler()
        self.create_interactive_marker()

    def avoid_collision(self):
        if (any([agent.position is None for agent in self.agents])):
            return

        obstacles = np.array(
            [agent.position for agent in self.agents if agent != self.agents[self.agent_index]])
        velocity = self.nmpc.step(
            self.agents[self.agent_index].position, obstacles)

        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = velocity[1]
        cmd_vel_msg.linear.y = -velocity[0]

        print(velocity)

        self.cmd_vel_pub.publish(cmd_vel_msg)

    def create_interactive_marker(self):
        interactive_marker = InteractiveMarker()
        interactive_marker.header.frame_id = "world"
        interactive_marker.name = self.node_name + "_goal_marker"
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
            interactive_marker, feedback_callback=self.goal_marker_feedback)
        self.menu_handler.apply(self.marker_server, interactive_marker.name)

        self.marker_server.applyChanges()

    def goal_marker_feedback(self, feedback):
        if feedback.event_type == feedback.POSE_UPDATE:
            self.nmpc.set_goal(
                (feedback.pose.position.x, feedback.pose.position.y))


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
