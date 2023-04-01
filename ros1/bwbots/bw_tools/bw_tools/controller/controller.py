from typing import Tuple
from bw_tools.robot_state import Pose2d, Velocity
from bw_tools.controller.data import ControllerStateMachineConfig

class Controller:
    def __init__(self,
                config: ControllerStateMachineConfig) -> None:
        self.config = config

    def compute(self, goal_pose: Pose2d, current_pose: Pose2d) -> Tuple[Velocity, bool]:
        """
        returns the best velocity to get to the goal pose from the current pose.
        this method also returns whether it's done or not.
        """
        return Velocity(), True
