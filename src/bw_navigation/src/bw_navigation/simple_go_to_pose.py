#!/usr/bin/env python3
from typing import Optional, Tuple

import rospy

from bw_navigation.navigation_action import NavigationAction
from bw_tools.controller.bwbots_controller import BwbotsController
from bw_tools.controller.data import ControllerStateMachineConfig, TrapezoidalProfileConfig
from bw_tools.robot_state import Pose2d, Pose2dStamped, Velocity
from bw_tools.structs.go_to_goal import GoToPoseGoal


class SimpleGoToPose(NavigationAction):
    def __init__(self) -> None:
        rospy.init_node(
            "simple_go_to_pose",
            log_level=rospy.DEBUG,
        )
        super().__init__()
        self.controller: Optional[BwbotsController] = None

    def controller_init(self, goal: GoToPoseGoal):
        xy_tolerance: float = goal.xy_tolerance
        yaw_tolerance: float = goal.yaw_tolerance
        reference_linear_speed: float = goal.reference_linear_speed
        reference_angular_speed: float = goal.reference_angular_speed
        allow_reverse = goal.allow_reverse
        rotate_in_place_start = goal.rotate_in_place_start
        rotate_while_driving = goal.rotate_while_driving
        rotate_in_place_end = goal.rotate_in_place_end
        linear_max_accel = goal.linear_max_accel
        theta_max_accel = goal.theta_max_accel
        linear_min_vel = goal.linear_min_vel
        theta_min_vel = goal.theta_min_vel

        linear_trapezoid = TrapezoidalProfileConfig(linear_min_vel, reference_linear_speed, linear_max_accel)
        angular_trapezoid = TrapezoidalProfileConfig(theta_min_vel, reference_angular_speed, theta_max_accel)
        pose_tolerance = Pose2d(xy_tolerance, xy_tolerance, yaw_tolerance)
        controller_config = ControllerStateMachineConfig(
            allow_reverse,
            rotate_while_driving,
            angular_trapezoid,
            linear_trapezoid,
            pose_tolerance,
            settle_time=self.settling_time,
            rotate_in_place_start=rotate_in_place_start,
            rotate_in_place_end=rotate_in_place_end,
            rotate_angle_threshold=self.rotate_angle_threshold,
            strafe_angle_threshold=self.strafe_angle_threshold,
        )

        self.controller = BwbotsController(controller_config)

    def controller_update(self, goal: GoToPoseGoal, robot_state: Pose2dStamped) -> Tuple[Velocity, bool]:
        assert self.controller is not None
        velocity_command, is_done = self.controller.compute(goal.goal.pose, robot_state.pose)
        return velocity_command, is_done


def main():
    node = SimpleGoToPose()
    node.run()


if __name__ == "__main__":
    main()
