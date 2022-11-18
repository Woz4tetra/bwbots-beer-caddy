import rospy
import py_trees_ros
from typing import Optional

from bw_interfaces.srv import GetWaypoints

from bw_interfaces.msg import FollowWaypointsAction, FollowWaypointsGoal


class FollowWaypointBehavior(py_trees_ros.actions.ActionClient):
    def __init__(self, waypoint_name):
        super().__init__("Follow Waypoint",
            FollowWaypointsAction,
            action_namespace="/bw/follow_waypoints")
        self.waypoint_name = waypoint_name
        self.get_waypoints_srv: Optional[rospy.ServiceProxy] = None

    def get_waypoint(self, name):
        rospy.loginfo(f"Requesting locations of waypoint: {name}")
        if self.get_waypoints_srv is None:
            rospy.logwarn("Waypoint service is not initialized!")
            return None
        result = self.get_waypoints_srv([name])
        if not result.success:
            rospy.logwarn("Failed to get waypoint locations")
            return None
        rospy.loginfo(f"Waypoint locations: {result}")
        waypoints_array = result.waypoints
        return waypoints_array

    def setup(self, timeout):
        self.get_waypoints_srv = rospy.ServiceProxy("/bw/bw_waypoints/get_waypoints", GetWaypoints)
        super().setup(timeout)

    def update(self):
        if not self.sent_goal:
            waypoint_array = self.get_waypoint(self.waypoint_name)
            self.action_goal = FollowWaypointsGoal()
            self.action_goal.waypoints = waypoint_array
        return super().update()
