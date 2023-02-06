from typing import Tuple
from bw_tools.robot_state import Pose2d, Velocity


class ControllerBehavior:
    def __init__(self) -> None:
        pass
    
    def initialize(self, goal_pose: Pose2d, current_pose: Pose2d) -> None:
        pass
    
    def compute(self, goal_pose: Pose2d, current_pose: Pose2d) -> Tuple[Velocity, bool]:
        return Velocity(), True
    
    def deinitialize(self, goal_pose: Pose2d, current_pose: Pose2d) -> None:
        pass
