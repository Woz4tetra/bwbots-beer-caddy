from abc import abstractmethod
from typing import Protocol

from bw_tools.robot_state import Pose2dStamped, Velocity
from bw_tools.structs.go_to_goal import GoToPoseGoal


class BaseOptimizer(Protocol):
    @abstractmethod
    def stop_command(self) -> Velocity:
        pass

    @abstractmethod
    def solve(self, goal: GoToPoseGoal, robot_state: Pose2dStamped) -> bool:
        pass

    @abstractmethod
    def update(self, goal: GoToPoseGoal, robot_state: Pose2dStamped) -> Velocity:
        pass
