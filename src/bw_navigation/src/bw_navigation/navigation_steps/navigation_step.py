from abc import abstractmethod
from typing import Protocol, Tuple

from bw_tools.robot_state import Pose2d, Velocity


class NavigationStep(Protocol):
    @abstractmethod
    def initialize(self) -> None:
        pass

    @abstractmethod
    def step(self, relative_robot_state: Pose2d) -> Tuple[Velocity, bool]:
        pass
