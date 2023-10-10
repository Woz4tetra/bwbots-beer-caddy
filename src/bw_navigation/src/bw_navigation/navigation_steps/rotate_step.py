from typing import Tuple

from bw_tools.controller.angle_wrap_manager import AngleWrapManager
from bw_tools.controller.tolerance_timer import ToleranceTimer
from bw_tools.controller.trapezoidal_profile import TrapezoidalProfile
from bw_tools.robot_state import Pose2d, Velocity

from .navigation_step import NavigationStep


class RotateStep(NavigationStep):
    def __init__(
        self, goal_angle: float, max_speed: float, min_speed: float, acceleration: float, angle_tolerance: float
    ) -> None:
        super().__init__()
        self.settle_time = 2.0
        self.angle_tolerance = angle_tolerance
        self.goal_angle = goal_angle
        self.timer = ToleranceTimer(self.settle_time)
        self.wrap_manager = AngleWrapManager()
        self.trapezoid = TrapezoidalProfile(max_speed, min_speed, acceleration)

    def initialize(self) -> None:
        self.timer.reset()
        self.wrap_manager.reset()

    def step(self, relative_robot_state: Pose2d) -> Tuple[Velocity, bool]:
        error = self.goal_angle - relative_robot_state.theta
        traveled = self.goal_angle - error
        angular_velocity = self.trapezoid.compute(self.goal_angle, traveled)
        is_in_tolerance = abs(error) < self.angle_tolerance
        if is_in_tolerance:
            velocity = Velocity()
        else:
            velocity = Velocity(0.0, 0.0, angular_velocity)
        return velocity, self.timer.is_done(is_in_tolerance)
