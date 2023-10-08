import dataclasses
from typing import cast

import rospy
from py_trees.behaviour import Behaviour
from py_trees.common import Status

from bw_behaviors.container import Container
from bw_tools.robot_state import Pose2d
from bw_tools.structs.go_to_goal import GoToPoseGoal


class SimpleGoToWaypoint(Behaviour):
    def __init__(self, container: Container, waypoint_name: str, relative_goal: GoToPoseGoal):
        super().__init__(self.__class__.__name__ + "-" + waypoint_name)
        self.relative_goal = relative_goal
        self.waypoint_name = waypoint_name
        self.go_to_pose = container.simple_go_to_pose
        self.waypoint_manager = container.waypoint_manager
        self.global_frame = container.global_frame
        self.is_goal_sent = True

    def initialise(self) -> None:
        self.is_goal_sent = False

    def update(self) -> Status:
        if not self.is_goal_sent:
            waypoint = self.waypoint_manager.waypoints.get(self.waypoint_name)
            if waypoint is None:
                rospy.logwarn(f"Waypoint {self.waypoint_name} not registered")
                return Status.FAILURE
            goal = dataclasses.replace(self.relative_goal)
            offset = goal.goal.pose  # the supplied relative goal is the goal
            waypoint_2d = waypoint.to_pose2d()  # convert waypoint to pose 2d

            # apply offset at the tip of the waypoint
            goal_2d = cast(Pose2d, offset.transform_by(waypoint_2d))

            # goal is now in the global frame
            goal.goal.pose = goal_2d
            goal.goal.header.frame_id = self.global_frame

            self.go_to_pose.send_goal(goal.to_msg())
            self.is_goal_sent = True

        return self.go_to_pose.status

    def terminate(self, new_status: Status) -> None:
        self.go_to_pose.cancel()
