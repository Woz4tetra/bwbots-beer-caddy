#!/usr/bin/env python3
from typing import Optional, Tuple

import rospy

from bw_navigation.navigation_action import NavigationAction
from bw_tools.controller.bwbots_controller import BwbotsController
from bw_tools.robot_state import Pose2d, Pose2dStamped, Velocity
from bw_tools.structs.go_to_goal import GoToPoseGoal


class OptimizerGoToPose(NavigationAction):
    def __init__(self) -> None:
        rospy.init_node(
            "optimizer_go_to_pose",
            log_level=rospy.DEBUG,
        )
        super().__init__()

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

    def controller_update(self, goal: GoToPoseGoal, robot_state: Pose2dStamped) -> Tuple[Velocity, bool]:
        assert self.controller is not None

        return velocity_command, is_done


def main():
    node = OptimizerGoToPose()
    node.run()


if __name__ == "__main__":
    main()
