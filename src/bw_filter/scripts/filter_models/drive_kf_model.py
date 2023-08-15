import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped

from tj2_tools.robot_state import Pose2d, Velocity

from .helpers import (
    jit_predict,
    jit_update,
    landmark_to_measurement,
    odometry_to_measurement,
    NUM_MEASUREMENTS,
    NUM_STATES,
    NUM_STATES_1ST_ORDER,
)
from .filter_model import FilterModel


class DriveKalmanModel(FilterModel):
    def __init__(self, dt: float) -> None:
        self.dt = dt
        self.state = np.zeros(NUM_STATES)
        self.covariance = np.eye(NUM_STATES)
        self.process_noise_Q = np.eye(NUM_STATES) * 0.0001

        # measurement function for landmarks. Use only position.
        self.landmark_H = np.zeros((NUM_MEASUREMENTS, NUM_STATES))
        self.landmark_H[0:NUM_STATES_1ST_ORDER, 0:NUM_STATES_1ST_ORDER] = np.eye(
            NUM_MEASUREMENTS
        )

        # measurement function for odometry. Use only velocity.
        self.odom_H = np.zeros((NUM_MEASUREMENTS, NUM_STATES))
        self.odom_H[0:NUM_STATES_1ST_ORDER, NUM_STATES_1ST_ORDER:NUM_STATES] = np.eye(
            NUM_MEASUREMENTS
        )

    def predict(self) -> None:
        self.state, self.covariance = jit_predict(
            self.state, self.covariance, self.process_noise_Q, self.dt
        )

    def update_landmark(self, msg: PoseWithCovarianceStamped) -> None:
        measurement, noise = landmark_to_measurement(msg)
        self.state, self.covariance = jit_update(
            self.state, self.covariance, self.landmark_H, measurement, noise
        )

    def update_odometry(self, msg: Odometry) -> None:
        measurement, noise = odometry_to_measurement(msg)
        self.state, self.covariance = jit_update(
            self.state, self.covariance, self.odom_H, measurement, noise
        )

    def get_pose(self) -> Pose2d:
        return Pose2d(self.state[0], self.state[1], self.state[2])

    def get_velocity(self) -> Velocity:
        return Velocity(self.state[3], self.state[4], self.state[5])

    def get_covariance(self) -> np.ndarray:
        return self.covariance

    def reset(self, msg: PoseWithCovarianceStamped) -> None:
        measurement, measurement_noise = landmark_to_measurement(msg)
        self.state = np.zeros(NUM_STATES)
        self.state[0:NUM_STATES_1ST_ORDER] = measurement
        self.covariance = np.eye(NUM_STATES)
        self.covariance[
            0:NUM_STATES_1ST_ORDER, 0:NUM_STATES_1ST_ORDER
        ] = measurement_noise
