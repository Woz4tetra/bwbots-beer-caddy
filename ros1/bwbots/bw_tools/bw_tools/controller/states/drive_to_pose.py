import math
import rospy
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
                angular_trapezoid: TrapezoidalProfileConfig,
                strafe_angle_threshold: float) -> None:
        self.rotate_while_driving = rotate_while_driving
        self.check_angle_tolerance = check_angle_tolerance
        self.settle_time = settle_time
        self.pose_tolerance = pose_tolerance
        self.linear_trapezoid_config = linear_trapezoid
        self.angular_trapezoid_config = angular_trapezoid
        self.angle_correction_kP = 0.2
        self.lateral_correction_kP = 0.2
        self.strafe_angle_threshold = strafe_angle_threshold

        self.linear_trapezoid: Optional[TrapezoidalProfile] = None
        self.angular_trapezoid: Optional[TrapezoidalProfile] = None

        self.timer = ToleranceTimer(settle_time)
        self.start_pose = Pose2d()

    def initialize(self, goal_pose: Pose2d, current_pose: Pose2d) -> None:
        self.linear_trapezoid = TrapezoidalProfile(self.linear_trapezoid_config)
        self.angular_trapezoid = TrapezoidalProfile(self.angular_trapezoid_config)
        self.timer.reset()
        self.start_pose = current_pose
        rospy.logdebug(f"Drive to pose initialized. Start: {self.start_pose}. Goal: {goal_pose}")

    def compute(self, goal_pose: Pose2d, current_pose: Pose2d) -> Tuple[Velocity, bool]:
        assert self.linear_trapezoid is not None, "Drive to pose not initialized!"
        assert self.angular_trapezoid is not None, "Drive to pose not initialized!"
        relative_goal = goal_pose.relative_to(self.start_pose)
        traveled = current_pose.relative_to(self.start_pose)

        error = goal_pose.relative_to(current_pose)
        heading = error.heading()
        
        distance_error = relative_goal.x - traveled.x
        lateral_error = relative_goal.y - traveled.y
        
        distance_in_tolerance = abs(distance_error) < self.pose_tolerance.x
        if self.check_angle_tolerance:
            yaw_in_tolerance = abs(heading) < self.pose_tolerance.theta
        else:
            yaw_in_tolerance = True
        
        if self.rotate_while_driving:
            angular_velocity = self.angle_correction_kP * heading + self.lateral_correction_kP * -lateral_error
        else:
            angular_velocity = 0.0
        
        forward_velocity = self.linear_trapezoid.compute(relative_goal.x, traveled.x)

        if distance_in_tolerance:
            forward_velocity = 0.0
        if yaw_in_tolerance:
            angular_velocity = 0.0

        return Velocity(forward_velocity, 0.0, angular_velocity), self.timer.is_done(distance_in_tolerance and yaw_in_tolerance)

    def deinitialize(self, goal_pose: Pose2d, current_pose: Pose2d) -> None:
        pass
