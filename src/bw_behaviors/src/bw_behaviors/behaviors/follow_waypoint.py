from typing import cast

import rospy
from py_trees.behaviour import Behaviour
from py_trees.common import Status

from bw_behaviors.container import Container
from bw_tools.robot_state import Pose2d


class FollowWaypoint(Behaviour):
    def __init__(self, container: Container, waypoint_name: str, offset: Pose2d = Pose2d()):
        super().__init__(self.__class__.__name__)
        self.offset = offset
        self.move_base = container.move_base_manager
        self.waypoint_manager = container.waypoint_manager
        self.waypoint_name = waypoint_name
        self.is_goal_sent = True

    def initialise(self) -> None:
        self.is_goal_sent = False

    def update(self) -> Status:
        if not self.is_goal_sent:
            waypoint = self.waypoint_manager.waypoints.get(self.waypoint_name)
            if waypoint is None:
                rospy.logwarn(f"Waypoint {self.waypoint_name} not registered")
                return Status.FAILURE
            goal_2d = waypoint.to_pose2d()
            goal_2d = cast(Pose2d, self.offset.transform_by(goal_2d))
            self.move_base.send_goal_2d(goal_2d)
            self.is_goal_sent = True
        if self.move_base.is_done():
            return Status.SUCCESS if self.move_base.did_succeed() else Status.FAILURE
        return Status.RUNNING

    def terminate(self, new_status: Status) -> None:
        self.move_base.cancel()
