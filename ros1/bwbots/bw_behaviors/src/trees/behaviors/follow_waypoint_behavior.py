from typing import Callable
import py_trees
import py_trees_ros
from bw_interfaces.msg import FollowWaypointsAction, FollowWaypointsGoal

from trees.managers.waypoint_manager import WaypointManager


class FollowWaypointBehavior(py_trees_ros.actions.ActionClient):
    def __init__(self, waypoint_name_supplier: Callable[[], str], waypoint_manager: WaypointManager):
        super().__init__("Follow Waypoint",
            FollowWaypointsAction,
            action_namespace="/bw/follow_waypoints")
        self.waypoint_name_supplier = waypoint_name_supplier
        self.waypoint_manager = waypoint_manager

    def update(self):
        if not self.sent_goal:
            waypoint_name = self.waypoint_name_supplier()
            if type(waypoint_name) != str:
                return py_trees.Status.FAILURE
            waypoint_array = self.waypoint_manager.get_waypoint(waypoint_name)
            if waypoint_array is None:
                return py_trees.Status.FAILURE
            self.action_goal = FollowWaypointsGoal()
            self.action_goal.waypoints = waypoint_array
        return super().update()
