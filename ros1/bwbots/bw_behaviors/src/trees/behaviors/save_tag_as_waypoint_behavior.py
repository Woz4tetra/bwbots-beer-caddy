
import rospy
import tf2_ros
import py_trees
import tf2_geometry_msgs
from typing import Optional

from bw_interfaces.srv import SaveWaypoint

from bw_interfaces.msg import Waypoint

from .util import get_offset_tag


class SaveTagAsWaypoint(py_trees.behaviour.Behaviour):
    def __init__(self, x_offset, y_offset, blackboard_name, global_frame_id):
        self.x_offset = x_offset
        self.y_offset = y_offset
        self.blackboard_name = blackboard_name
        self.global_frame_id = global_frame_id
        self.save_waypoint_srv: Optional[rospy.ServiceProxy] = None
        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

    def setup(self, timeout):
        self.save_waypoint_srv = rospy.ServiceProxy("/bw/bw_waypoints/save_waypoint", SaveWaypoint)
        super().setup(timeout)

    def update(self):
        if self.save_waypoint_srv is None:
            raise RuntimeError("Failed to initialize save waypoint service")
        dock_tag_pose_stamped = get_offset_tag(self.blackboard.get(self.blackboard_name), self.x_offset, self.y_offset)
        try:
            transform = self.tf_buffer.lookup_transform(self.global_frame_id, dock_tag_pose_stamped.header.frame_id, rospy.Time(0), rospy.Duration(1.0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn(f"Failed to lookup global to tag frame: {e}")
            return py_trees.Status.FAILURE

        tag_waypoint = tf2_geometry_msgs.do_transform_pose(dock_tag_pose_stamped, transform)

        waypoint = Waypoint()
        waypoint.name = self.blackboard_name
        waypoint.header.frame_id = self.global_frame_id
        waypoint.pose = tag_waypoint
        result = self.save_waypoint_srv(waypoint)
        if not result.success:
            rospy.logwarn("Failed to save tag waypoint")
            return py_trees.Status.FAILURE
        else:
            return py_trees.Status.SUCCESS
