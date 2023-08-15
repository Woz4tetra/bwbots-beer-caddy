import numpy as np
from typing import Tuple
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped

from bw_tools.robot_state import Pose2d, Velocity
from abc import abstractmethod


class FilterModel:
    @abstractmethod
    def predict(self) -> None:
        pass

    @abstractmethod
    def update_landmark(self, msg: PoseWithCovarianceStamped) -> None:
        pass

    @abstractmethod
    def update_odometry(self, msg: Odometry) -> None:
        pass

    @abstractmethod
    def get_pose(self) -> Pose2d:
        pass

    @abstractmethod
    def get_velocity(self) -> Velocity:
        pass

    def get_state(self) -> Tuple[Pose2d, Velocity]:
        return self.get_pose(), self.get_velocity()

    @abstractmethod
    def get_covariance(self) -> np.ndarray:
        pass

    @abstractmethod
    def reset(self, msg: PoseWithCovarianceStamped) -> None:
        pass
