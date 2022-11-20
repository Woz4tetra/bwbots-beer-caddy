import py_trees_ros
from bw_interfaces.msg import FollowWaypointsAction, FollowWaypointsGoal

from trees.managers.waypoint_manager import WaypointManager


class FollowWaypointBehavior(py_trees_ros.actions.ActionClient):
    def __init__(self, waypoint_name, waypoint_manager: WaypointManager):
        super().__init__("Follow Waypoint",
            FollowWaypointsAction,
            action_namespace="/bw/follow_waypoints")
        self.waypoint_name = waypoint_name
        self.waypoint_manager = waypoint_manager

    def update(self):
        if not self.sent_goal:
            waypoint_array = self.waypoint_manager.get_waypoint(self.waypoint_name)
            self.action_goal = FollowWaypointsGoal()
            self.action_goal.waypoints = waypoint_array
        return super().update()
