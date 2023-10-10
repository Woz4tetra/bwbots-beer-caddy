from typing import List, Tuple

import numpy as np
import rospy
from scipy.interpolate import splev, splprep

from bw_tools.controller import PIDController
from bw_tools.robot_state import Pose2d, Velocity
from bw_tools.typing.basic import seconds_to_time

from .navigation_step import NavigationStep


class FollowSplineStep(NavigationStep):
    def __init__(self, relative_goal: Pose2d, max_velocity: float, max_accel: float) -> None:
        super().__init__()
        self.spline_samples = 100
        self.end_time_buffer = 1.0
        self.max_velocity = max_velocity
        self.max_accel = max_accel
        self.path, self.control_pts = self.generate_path(
            (0.0, 0.0), (relative_goal.x, relative_goal.y), self.spline_samples
        )
        delta_distances = np.linalg.norm(np.diff(self.path, axis=1), axis=0)
        delta_distances = np.insert(delta_distances, 0, 0.0)
        path_distance = np.cumsum(delta_distances)

        vxs = self.compute_velocity(self.path[0], self.max_velocity, self.max_accel)
        vys = self.compute_velocity(self.path[1], self.max_velocity, self.max_accel)

        self.velocities = np.array([vxs, vys], dtype=np.float64)
        speeds = np.linalg.norm(self.velocities, axis=1)

        total_time = path_distance[-1] / np.mean(speeds)
        total_time += self.end_time_buffer
        self.times = np.linspace(0.0, total_time, len(speeds))

        self.start_time = seconds_to_time(0.0)
        self.prev_time = seconds_to_time(0.0)

        kp = 1.0
        ki = 0.0
        kd = 0.0
        self.x_control = PIDController(kp=kp, ki=ki, kd=kd)
        self.y_control = PIDController(kp=kp, ki=ki, kd=kd)

    def initialize(self) -> None:
        self.start_time = rospy.Time.now()
        self.prev_time = self.start_time

    def step(self, relative_robot_state: Pose2d) -> Tuple[Velocity, bool]:
        now = rospy.Time.now()
        relative_time = (now - self.start_time).to_sec()
        delta_time = (now - self.prev_time).to_sec()
        self.prev_time = now

        if relative_time > self.times[-1]:
            return Velocity(), True

        index = self.get_nearest_index(relative_time)
        velocity = Velocity(self.velocities[0, index], self.velocities[1, index], theta=0.0)
        waypoint = Pose2d(self.path[0, index], self.path[1, index], theta=0.0)

        velocity.x += self.x_control.update(waypoint.x, relative_robot_state.x, delta_time)
        velocity.y += self.x_control.update(waypoint.y, relative_robot_state.y, delta_time)

        return velocity, False

    def get_nearest_index(self, relative_time: float) -> int:
        return np.argmin(np.abs(self.times - relative_time))

    def generate_path(
        self,
        start: Tuple[float, float],
        stop: Tuple[float, float],
        num_samples: int,
    ) -> Tuple[np.ndarray, np.ndarray]:
        points: List[Tuple[float, float]] = []
        mid1 = self.get_mid_point(start, stop, 0.75)
        mid2 = self.get_mid_point(start, stop, 0.25)
        mid3 = self.get_mid_point(start, stop, 0.9)
        mid4 = self.get_mid_point(start, stop, 0.1)

        points.append((start[0], start[1]))
        points.append((mid1[0], mid3[1]))
        points.append((mid2[0], mid4[1]))
        points.append((stop[0], stop[1]))
        control_pts = np.array(points, dtype=np.float64).T
        x = control_pts[0]
        y = control_pts[1]
        path = self.make_path_spline(x, y, num_samples)
        return path, control_pts

    def get_mid_point(self, start: Tuple[float, float], stop: Tuple[float, float], weight: float) -> np.ndarray:
        assert 0.0 <= weight <= 1.0, weight
        samples = np.append([start], [stop], axis=0)
        return np.array(np.average(samples, axis=0, weights=[weight, 1.0 - weight]))

    def make_path_spline(
        self,
        x: np.ndarray,
        y: np.ndarray,
        num_samples: int,
        spline_degree: int = 3,
        smoothing_condition: float = 1,
    ) -> np.ndarray:
        tck, _ = splprep([x, y], k=spline_degree, s=smoothing_condition)
        u = np.linspace(0.0, 1.0, num=num_samples, endpoint=True)
        xy_path = splev(u, tck)
        xy_path = np.array(xy_path, dtype=np.float64)
        xy_path[0, 0] = x[0]
        xy_path[1, 0] = y[0]
        xy_path[0, -1] = x[-1]
        xy_path[1, -1] = y[-1]
        return xy_path

    def compute_velocity(
        self,
        path_single_dim: np.ndarray,
        target_velocity: float,
        target_acceleration: float,
        no_motion_distance: float = 1e-3,
        epsilon: float = 1e-6,
    ) -> np.ndarray:
        total_distance = path_single_dim[-1]
        max_vel = np.sqrt(total_distance * target_acceleration)
        velocity = np.linspace(0.0, max_vel, len(path_single_dim) // 2)
        velocity = np.append(velocity, np.linspace(max_vel, 0.0, len(path_single_dim) - len(velocity)))
        velocity -= np.sqrt(no_motion_distance * target_acceleration)
        velocity = np.clip(velocity, 0.0, target_velocity)
        if np.all(velocity < epsilon):
            velocity[:] = epsilon
        return np.array(velocity, dtype=np.float64)
