from typing import Optional, Tuple
from bw_tools.robot_state import Pose2d, Velocity
from bw_tools.controller.data import TrapezoidalProfileConfig
from bw_tools.controller.trapezoidal_profile import TrapezoidalProfile
from bw_tools.controller.states.controller_behavior import ControllerBehavior
from bw_tools.controller.states.tolerance_timer import ToleranceTimer


class RotateInPlace(ControllerBehavior):
    def __init__(self, settle_time: float, angle_tolerance: float, trapezoid: TrapezoidalProfileConfig) -> None:
        self.angle_tolerance = angle_tolerance
        self.trapezoid_config = trapezoid
        self.trapezoid: Optional[TrapezoidalProfile] = None
        self.timer = ToleranceTimer(settle_time)
    
    def initialize(self, goal_pose: Pose2d, current_pose: Pose2d) -> None:
        self.trapezoid = TrapezoidalProfile(self.trapezoid_config)
        self.timer.reset()

    def compute(self, goal_pose: Pose2d, current_pose: Pose2d) -> Tuple[Velocity, bool]:
        assert self.trapezoid is not None, "Rotate in place not initialized!"
        goal = goal_pose.theta
        current = current_pose.theta
        angular_velocity = self.trapezoid.compute(goal, current)
        error = goal - current
        velocity = Velocity(0.0, 0.0, angular_velocity)
        return velocity, self.timer.is_done(abs(error) < angular_velocity)
    
    def deinitialize(self, goal_pose: Pose2d, current_pose: Pose2d) -> None:
        pass
