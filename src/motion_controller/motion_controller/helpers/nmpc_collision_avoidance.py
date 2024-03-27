"""
Collision avoidance using Nonlinear Model-Predictive Control

Adapted from Ashwin Bose 
https://github.com/atb033/multi_agent_path_planning/blob/master/decentralized/nmpc/nmpc.py
"""

from typing import Tuple
import numpy as np
from scipy.optimize import minimize, Bounds


class Nmpc():

    def __init__(self, robot_radius, vmax, vmin, timestep=0.1, nmpc_timestep=0.3,  horizon_length=int(4), static_obstacles=[]):
        self.timestep = timestep
        self.robot_radius = robot_radius
        self.vmax = vmax
        self.vmin = vmin

        # collision cost parameters
        # https://www.desmos.com/calculator/lu9hv6mq36
        # Assumes a agent radius of 0.25, we adjust scale to set actual agent radius
        self.Qc = 8.
        self.kappa = 6.
        self.static_kappa = 6.
        self.scale = robot_radius/0.25
        self.robot_radius = robot_radius
        self.latency = latency

        self.last_velocity_profile = None

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

        # line segments (x1, y1, x2, y2)
        self.static_obstacles = []

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
        u0 = (2*np.random.rand(2*self.horizon_length) - 1) * self.vmax
        # if self.last_velocity_profile is None:
        #     u0 = np.array([0] * 2 * self.horizon_length)
        # else:
        #     u0 = np.append(self.last_velocity_profile[2:], [0, 0])

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
                cost = self.scale * self.Qc / \
                    (1 + np.exp(self.static_kappa *
                     distance / (self.scale/2)))
                total_cost += cost
        return total_cost

    def collision_cost(self, x0, x1):
        """
        Cost of collision between two robot_state
        """
        d = np.linalg.norm(x0 - x1)
        cost = self.scale * self.Qc / \
            (1 + np.exp(self.kappa * d / self.scale))
        return cost

    def distance_point_to_line_segment(self, point, line_segment_points):
        # Extracting coordinates
        x1, y1, x2, y2 = line_segment_points
        line_segment_start = np.array([x1, y1])
        line_segment_end = np.array([x2, y2])
        point = np.array(point)

        # Vector representing the line segment
        v = line_segment_end - line_segment_start

        # Vector from line_segment_start to point
        w = point - line_segment_start

        # Dot product of v and w
        dot_product = np.dot(w, v)

        # Clamp dot product within the bounds of the line segment
        dot_product = np.clip(dot_product, 0, np.dot(v, v))

        # Closest point on the line segment to point
        closest_point = line_segment_start + dot_product * v / np.dot(v, v)

        # Calculate distance between point and closest_point
        distance = np.linalg.norm(closest_point - point)

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
