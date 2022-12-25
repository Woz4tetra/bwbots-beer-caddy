
import rospy
import tf2_ros
import py_trees
import tf2_geometry_msgs
from typing import Callable, Optional

from bw_interfaces.srv import SaveWaypoint

from bw_interfaces.msg import Waypoint

from trees.managers.tag_manager import TagManager


class SaveTagAsWaypointBehavior(py_trees.behaviour.Behaviour):
    def __init__(self,
                x_offset: float,
                y_offset: float,
                theta_offset: float,
                global_frame_id: str,
                waypoint_name_supplier: Callable[[], str],
                tag_name_supplier: Callable[[], str],
                tag_manager: TagManager):
        self.x_offset = x_offset
        self.y_offset = y_offset
        self.theta_offset = theta_offset
        self.tag_name_supplier = tag_name_supplier
        self.waypoint_name_supplier = waypoint_name_supplier
        self.tag_manager = tag_manager
        self.global_frame_id = global_frame_id
        self.save_waypoint_srv = rospy.ServiceProxy("/bw/bw_waypoints/save_waypoint", SaveWaypoint)
        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        super().__init__("Save tag as waypoint")

    def update(self):
        if self.save_waypoint_srv is None:
            raise RuntimeError("Failed to initialize save waypoint service")
        tag_name = self.tag_name_supplier()
        if type(tag_name) != str:
            return py_trees.Status.FAILURE
        tag_pose_stamped = self.tag_manager.get_offset_tag(tag_name, self.x_offset, self.y_offset, self.theta_offset)
        if tag_pose_stamped is None:
            return py_trees.Status.FAILURE
        try:
            transform = self.tf_buffer.lookup_transform(self.global_frame_id, tag_pose_stamped.header.frame_id, rospy.Time(0), rospy.Duration(1.0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn(f"Failed to lookup global to tag frame: {e}")
            return py_trees.Status.FAILURE

        tag_waypoint = tf2_geometry_msgs.do_transform_pose(tag_pose_stamped, transform)

        waypoint_name = self.waypoint_name_supplier()
        if type(waypoint_name) != str:
            return py_trees.Status.FAILURE

        waypoint = Waypoint()
        waypoint.name = waypoint_name
        waypoint.header.frame_id = self.global_frame_id
        waypoint.pose = tag_waypoint.pose
        result = self.save_waypoint_srv(waypoint)
        if not result.success:
            rospy.logwarn("Failed to save tag waypoint")
            return py_trees.Status.FAILURE
        else:
            return py_trees.Status.SUCCESS
