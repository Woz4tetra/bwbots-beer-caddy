import math
import rospy
from typing import Optional, Tuple
from bw_tools.robot_state import Pose2d, Velocity
from bw_tools.controller.data import TrapezoidalProfileConfig
from bw_tools.controller.trapezoidal_profile import TrapezoidalProfile
from bw_tools.controller.states.controller_behavior import ControllerBehavior
from bw_tools.controller.states.tolerance_timer import ToleranceTimer
from bw_tools.controller.states.angle_wrap_manager import AngleWrapManager


class RotateToHeading(ControllerBehavior):
    def __init__(self, settle_time: float, angle_tolerance: float, trapezoid: TrapezoidalProfileConfig) -> None:
        self.angle_tolerance = angle_tolerance
        self.trapezoid_config = trapezoid
        self.trapezoid: Optional[TrapezoidalProfile] = None
        self.timer = ToleranceTimer(settle_time)
        self.is_already_at_goal = False
        self.goal_heading = None
        self.wrap_manager = AngleWrapManager()
    
    def initialize(self, goal_pose: Pose2d, current_pose: Pose2d) -> None:
        self.trapezoid = TrapezoidalProfile(self.trapezoid_config)
        self.timer.reset()
        self.is_already_at_goal = self.is_in_tolerance(goal_pose, current_pose)
        self.wrap_manager.reset()
        self.goal_heading = self.get_error(goal_pose, current_pose)
        rospy.logdebug(f"Rotate to heading initialized. Goal heading: {self.goal_heading}")
    
    def get_error(self, goal_pose: Pose2d, current_pose: Pose2d) -> float:
        goal_heading = goal_pose.relative_to(current_pose).heading()
        return self.wrap_manager.unwrap(goal_heading)

    def is_in_tolerance(self, goal_pose: Pose2d, current_pose: Pose2d) -> bool:
        error = self.get_error(goal_pose, current_pose)
        return abs(error) < self.angle_tolerance

    def compute(self, goal_pose: Pose2d, current_pose: Pose2d) -> Tuple[Velocity, bool]:
        assert self.trapezoid is not None, "Rotate to heading not initialized!"
        assert self.goal_heading is not None, "Rotate to heading not initialized!"
        if self.is_already_at_goal:
            return Velocity(), True
        traveled = self.goal_heading - self.get_error(goal_pose, current_pose)
        angular_velocity = self.trapezoid.compute(self.goal_heading, traveled)
        is_in_tolerance = self.is_in_tolerance(goal_pose, current_pose)
        if is_in_tolerance:
            velocity = Velocity()
        else:
            velocity = Velocity(0.0, 0.0, angular_velocity)
        return velocity, self.timer.is_done(is_in_tolerance)
