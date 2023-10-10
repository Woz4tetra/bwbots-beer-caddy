import math
from typing import List, Optional, Tuple, cast

from bw_navigation.base_optimizer import BaseOptimizer
from bw_navigation.navigation_steps import NavigationStep
from bw_navigation.navigation_steps.follow_spline_step import FollowSplineStep
from bw_navigation.navigation_steps.rotate_step import RotateStep
from bw_tools.robot_state import Pose2d, Pose2dStamped, Velocity
from bw_tools.structs.go_to_goal import GoToPoseGoal


class SplineOptimizer(BaseOptimizer):
    def __init__(self) -> None:
        self.max_strafe_angle = math.pi / 4.0
        self.prev_goal: Optional[GoToPoseGoal] = None
        self.start_pose: Pose2dStamped = Pose2dStamped.from_xyt(0.0, 0.0, 0.0)
        self.navigation_steps: List[NavigationStep] = []
        self.active_navigation_step: Optional[NavigationStep] = None

    def stop_command(self) -> Velocity:
        return Velocity()

    def compute_relative_goal(self, goal: GoToPoseGoal, robot_state: Pose2dStamped) -> Pose2d:
        return cast(Pose2d, goal.goal.pose.relative_to(robot_state.pose))

    def solve(self, goal: GoToPoseGoal, robot_state: Pose2dStamped) -> bool:
        self.start_pose = robot_state
        self.navigation_steps = []
        relative_goal = self.compute_relative_goal(goal, robot_state)

        heading = relative_goal.heading()
        end_angle = relative_goal.theta
        is_goal_behind = abs(heading) > math.pi / 2.0
        if is_goal_behind:
            heading = self.bound_angle(heading + math.pi)

        state_heading_aligned = Pose2dStamped.from_other(robot_state)
        state_heading_aligned.pose.theta = heading
        goal_heading_aligned = self.compute_relative_goal(goal, state_heading_aligned)

        state_end_aligned = Pose2dStamped.from_other(robot_state)
        state_end_aligned.pose.theta = end_angle
        goal_end_aligned = self.compute_relative_goal(goal, state_end_aligned)

        is_heading_strafeable = self.is_angle_strafeable(heading)
        is_end_strafeable = self.is_angle_strafeable(end_angle)
        is_end_strafeable_after_rotate = self.is_angle_strafeable(end_angle - heading)

        if is_heading_strafeable and is_end_strafeable:
            self.navigation_steps.append(self.get_drive_step(relative_goal, goal))
        elif is_heading_strafeable and not is_end_strafeable and is_end_strafeable_after_rotate:
            self.navigation_steps.append(self.get_rotate_step(end_angle, goal))
            self.navigation_steps.append(self.get_drive_step(goal_end_aligned, goal))
        else:
            self.navigation_steps.append(self.get_rotate_step(heading, goal))
            self.navigation_steps.append(self.get_drive_step(goal_heading_aligned, goal))
            self.navigation_steps.append(self.get_rotate_step(end_angle, goal))
        return True

    def update(self, goal: GoToPoseGoal, robot_state: Pose2dStamped) -> Tuple[Velocity, bool]:
        if self.prev_goal is None or self.prev_goal.goal != goal.goal:
            self.prev_goal = goal
            if not self.solve(goal, robot_state):
                return self.stop_command(), True
        if self.active_navigation_step is None:
            if len(self.navigation_steps) == 0:
                return self.stop_command(), True
            self.active_navigation_step = self.navigation_steps.pop(0)
            self.active_navigation_step.initialize()
        relative_robot_pose = cast(Pose2d, robot_state.pose.relative_to(self.start_pose.pose))
        command, is_done = self.active_navigation_step.step(relative_robot_pose)
        if is_done:
            self.active_navigation_step = None
        return self.get_strafe_limited_velocity(command), False

    def get_drive_step(self, relative_goal: Pose2d, goal: GoToPoseGoal) -> NavigationStep:
        return FollowSplineStep(
            relative_goal,
            goal.reference_linear_speed,
            goal.linear_max_accel,
        )

    def get_rotate_step(self, relative_goal: float, goal: GoToPoseGoal) -> NavigationStep:
        return RotateStep(
            relative_goal,
            goal.reference_angular_speed,
            goal.angle_min_vel,
            goal.angle_max_accel,
            goal.angle_tolerance,
        )

    def limit_left_half_circle(self, angle: float) -> float:
        angle += math.pi
        angle %= math.pi
        angle -= math.pi
        return angle

    def is_angle_strafeable(self, angle: float) -> bool:
        return abs(self.limit_left_half_circle(angle)) <= self.max_strafe_angle

    def bound_angle(self, angle: float) -> float:
        angle %= 2.0 * math.pi
        if angle > math.pi:
            angle -= 2.0 * math.pi
        if angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def get_strafe_limited_angle(self, angle: float) -> float:
        angle = self.bound_angle(angle)
        if -math.pi / 2.0 < angle < math.pi / 2.0:
            return self.clamp(angle, -self.max_strafe_angle, self.max_strafe_angle)
        elif angle >= math.pi / 2.0:
            return self.clamp(angle, math.pi - self.max_strafe_angle, math.pi)
        elif angle <= -math.pi / 2.0:
            return self.clamp(angle, -math.pi, -math.pi + self.max_strafe_angle)
        else:
            return angle

    def get_strafe_limited_velocity(self, velocity: Velocity) -> Velocity:
        angle = velocity.heading()
        limited_angle = self.get_strafe_limited_angle(angle)
        if angle != limited_angle:
            magnitude = velocity.magnitude()
            return cast(Velocity, Velocity(x=magnitude, y=0.0).rotate_by(limited_angle))
        else:
            return velocity

    def clamp(self, value: float, min_value: float, max_value: float) -> float:
        return max(min_value, min(max_value, value))
