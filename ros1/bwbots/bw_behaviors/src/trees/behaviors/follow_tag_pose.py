import py_trees
import py_trees_ros

from bw_interfaces.msg import FollowWaypointsAction, FollowWaypointsGoal
from bw_interfaces.msg import WaypointArray
from bw_interfaces.msg import Waypoint

from .util import get_offset_tag


class FollowTagBehavior(py_trees_ros.actions.ActionClient):
    def __init__(self, x_offset, y_offset, blackboard_name):
        self.x_offset = x_offset
        self.y_offset = y_offset
        self.blackboard_name = blackboard_name
        self.blackboard = py_trees.blackboard.Blackboard()

        super().__init__("Follow Tag",
            FollowWaypointsAction,
            action_namespace="/bw/follow_waypoints")

    def setup(self, timeout):
        return super().setup(timeout)

    def update(self):
        if not self.sent_goal:
            dock_tag_pose_stamped = get_offset_tag(self.blackboard.get(self.blackboard_name), self.x_offset, self.y_offset)

            waypoint_array = WaypointArray()

            tag_waypoint = Waypoint()
            tag_waypoint.name = self.blackboard_name
            tag_waypoint.header = dock_tag_pose_stamped.header
            tag_waypoint.pose = dock_tag_pose_stamped.pose
            waypoint_array.waypoints.append(tag_waypoint)

            self.action_goal = FollowWaypointsGoal()
            self.action_goal.waypoints = waypoint_array
