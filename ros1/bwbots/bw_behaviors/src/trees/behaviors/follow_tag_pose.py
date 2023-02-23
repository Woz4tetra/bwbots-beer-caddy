import rospy
from typing import Callable
import py_trees
import py_trees_ros

from bw_interfaces.msg import FollowWaypointsAction, FollowWaypointsGoal
from bw_interfaces.msg import WaypointArray
from bw_interfaces.msg import Waypoint

from trees.managers.tag_manager import TagManager


class FollowTagBehavior(py_trees_ros.actions.ActionClient):
    def __init__(self, x_offset: float, y_offset: float, theta_offset: float, tag_name_supplier: Callable[[], str], tag_manager: TagManager):
        self.x_offset = x_offset
        self.y_offset = y_offset
        self.theta_offset = theta_offset
        self.tag_name_supplier = tag_name_supplier
        self.tag_manager = tag_manager

        super().__init__("Follow Tag",
            FollowWaypointsAction,
            action_namespace="/bw/follow_waypoints")

    def update(self):
        if not self.sent_goal:
            tag_name = self.tag_name_supplier()
            rospy.loginfo(f"Following tag {tag_name}")
            if type(tag_name) != str:
                rospy.logwarn(f"Invalid tag name supplied: {tag_name}")
                return py_trees.Status.FAILURE
            
            tag_pose_stamped = self.tag_manager.get_offset_tag(tag_name, self.x_offset, self.y_offset, self.theta_offset)
            if tag_pose_stamped is None:
                rospy.logwarn(f"Tag name {tag_name} doesn't evaluate to a position")
                return py_trees.Status.FAILURE

            waypoint_array = WaypointArray()

            tag_waypoint = Waypoint()
            tag_waypoint.name = tag_name
            tag_waypoint.header = tag_pose_stamped.header
            tag_waypoint.pose = tag_pose_stamped.pose
            waypoint_array.waypoints.append(tag_waypoint)

            self.action_goal = FollowWaypointsGoal()
            self.action_goal.waypoints = waypoint_array
        action_result = super().update()
        return action_result
