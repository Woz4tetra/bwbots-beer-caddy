import py_trees
import py_trees_ros

from bw_interfaces.msg import FollowWaypointsAction, FollowWaypointsGoal
from bw_interfaces.msg import WaypointArray
from bw_interfaces.msg import Waypoint

from trees.managers.tag_manager import TagManager


class FollowTagBehavior(py_trees_ros.actions.ActionClient):
    def __init__(self, x_offset: float, y_offset: float, tag_name: str, tag_manager: TagManager):
        self.x_offset = x_offset
        self.y_offset = y_offset
        self.tag_name = tag_name
        self.tag_manager = tag_manager

        super().__init__("Follow Tag",
            FollowWaypointsAction,
            action_namespace="/bw/follow_waypoints")

    def setup(self, timeout):
        return super().setup(timeout)

    def update(self):
        if not self.sent_goal:
            tag_pose_stamped = self.tag_manager.get_offset_tag(self.tag_name, self.x_offset, self.y_offset)
            if tag_pose_stamped is None:
                return py_trees.Status.FAILURE

            waypoint_array = WaypointArray()

            tag_waypoint = Waypoint()
            tag_waypoint.name = self.tag_name
            tag_waypoint.header = tag_pose_stamped.header
            tag_waypoint.pose = tag_pose_stamped.pose
            waypoint_array.waypoints.append(tag_waypoint)

            self.action_goal = FollowWaypointsGoal()
            self.action_goal.waypoints = waypoint_array
