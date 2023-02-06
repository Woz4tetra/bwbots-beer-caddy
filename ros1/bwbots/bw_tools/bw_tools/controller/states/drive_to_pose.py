import math
from typing import Optional, Tuple
from bw_tools.controller.data import TrapezoidalProfileConfig
from bw_tools.controller.trapezoidal_profile import TrapezoidalProfile
from bw_tools.robot_state import Pose2d, Velocity
from bw_tools.controller.states.controller_behavior import ControllerBehavior
from bw_tools.controller.states.tolerance_timer import ToleranceTimer


class DriveToPose(ControllerBehavior):
    def __init__(self,
                rotate_while_driving: bool,
                check_angle_tolerance: bool,
                settle_time: float,
                pose_tolerance: Pose2d,
                linear_trapezoid: TrapezoidalProfileConfig,
                angular_trapezoid: TrapezoidalProfileConfig) -> None:
        self.rotate_while_driving = rotate_while_driving
        self.check_angle_tolerance = check_angle_tolerance
        self.settle_time = settle_time
        self.pose_tolerance = pose_tolerance
        self.linear_trapezoid_config = linear_trapezoid
        self.angular_trapezoid_config = angular_trapezoid

        self.linear_trapezoid: Optional[TrapezoidalProfile] = None
        self.angular_trapezoid: Optional[TrapezoidalProfile] = None

        self.timer = ToleranceTimer(settle_time)

    def initialize(self, goal_pose: Pose2d, current_pose: Pose2d) -> None:
        self.linear_trapezoid = TrapezoidalProfile(self.linear_trapezoid_config)
        self.angular_trapezoid = TrapezoidalProfile(self.angular_trapezoid_config)
        self.timer.reset()

    def compute(self, goal_pose: Pose2d, current_pose: Pose2d) -> Tuple[Velocity, bool]:
        assert self.linear_trapezoid is not None, "Drive to pose not initialized!"
        assert self.angular_trapezoid is not None, "Drive to pose not initialized!"
        error = goal_pose.relative_to(current_pose)
        heading = error.heading()
        distance = error.magnitude()
        
        if self.rotate_while_driving:
            angular_velocity = self.angular_trapezoid.compute(goal_pose.theta, current_pose.theta)
        else:
            angular_velocity = 0.0
        
        linear_velocity = self.linear_trapezoid.compute(distance, 0.0)
        
        vx = linear_velocity * math.cos(heading)
        vy = linear_velocity * math.sin(heading)
        
        xy_in_tolerance = distance < self.pose_tolerance.magnitude()
        if self.check_angle_tolerance:
            yaw_in_tolerance = abs(error.theta) < self.pose_tolerance.theta
        else:
            yaw_in_tolerance = True
        
        return Velocity(vx, vy, angular_velocity), self.timer.is_done(xy_in_tolerance and yaw_in_tolerance)

    def deinitialize(self, goal_pose: Pose2d, current_pose: Pose2d) -> None:
        pass
