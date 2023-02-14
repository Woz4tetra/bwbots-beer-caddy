import math
import rospy
from typing import Callable, Optional, Tuple
from bw_tools.robot_state import Pose2d, Velocity
from bw_tools.controller.data import TrapezoidalProfileConfig
from bw_tools.controller.trapezoidal_profile import TrapezoidalProfile
from bw_tools.controller.states.controller_behavior import ControllerBehavior
from bw_tools.controller.states.tolerance_timer import ToleranceTimer
from bw_tools.controller.states.angle_wrap_manager import AngleWrapManager


class RotateToAngle(ControllerBehavior):
    def __init__(self, 
            settle_time: float,
            angle_tolerance: float,
            trapezoid: TrapezoidalProfileConfig,
            angle_supplier: Callable) -> None:
        self.angle_tolerance = angle_tolerance
        self.trapezoid_config = trapezoid
        self.trapezoid: Optional[TrapezoidalProfile] = None
        self.timer = ToleranceTimer(settle_time)
        self.is_already_at_goal = False
        self.goal_angle = None
        self.wrap_manager = AngleWrapManager()
        self.angle_supplier = angle_supplier
    
    def initialize(self, goal_pose: Pose2d, current_pose: Pose2d) -> None:
        self.trapezoid = TrapezoidalProfile(self.trapezoid_config)
        self.timer.reset()
        error = self.get_error(goal_pose, current_pose)
        self.is_already_at_goal = abs(error) < self.angle_tolerance
        self.goal_angle = error
        self.wrap_manager.reset()
        rospy.logdebug(f"Rotate to angle initialized. Goal angle: {self.goal_angle}")
    
    def get_error(self, goal_pose: Pose2d, current_pose: Pose2d) -> float:
        goal_angle = self.angle_supplier(goal_pose, current_pose)
        return self.wrap_manager.unwrap(goal_angle)

    def compute(self, goal_pose: Pose2d, current_pose: Pose2d) -> Tuple[Velocity, bool]:
        assert self.trapezoid is not None, "Rotate to angle not initialized!"
        assert self.goal_angle is not None, "Rotate to angle not initialized!"
        if self.is_already_at_goal:
            return Velocity(), True
        error = self.get_error(goal_pose, current_pose)
        traveled = self.goal_angle - error
        angular_velocity = self.trapezoid.compute(self.goal_angle, traveled)
        is_in_tolerance = abs(error) < self.angle_tolerance
        if is_in_tolerance:
            velocity = Velocity()
        else:
            velocity = Velocity(0.0, 0.0, angular_velocity)
        return velocity, self.timer.is_done(is_in_tolerance)
