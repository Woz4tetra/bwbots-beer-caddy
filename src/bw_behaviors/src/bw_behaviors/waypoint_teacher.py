#!/usr/bin/env python3

import rospy
import tf2_ros
from geometry_msgs.msg import Pose, PoseStamped

from bw_behaviors.managers.named_offsets_manager import NamedOffsetsManager
from bw_interfaces.msg import Waypoint
from bw_interfaces.srv import SaveTF, SaveWaypoint, TeachWaypoint, TeachWaypointRequest, TeachWaypointResponse
from bw_tags.srv import RequestTags
from bw_tools.tags.tag_manager import TagManager
from bw_tools.transforms import lookup_pose_in_frame, transform_pose
from bw_tools.typing.basic import get_param


class WaypointTeacher:
    def __init__(self) -> None:
        rospy.init_node("waypoint_teacher")

        self.map_frame = get_param("~map_frame", "map")
        self.base_frame = get_param("~base_frame", "base_link")
        self.apriltag_config_path = get_param("~apriltag_config_path", "tags.yaml")
        self.named_offsets = NamedOffsetsManager(get_param("~offsets", None))

        self.tag_manager = TagManager(self.apriltag_config_path)

        self.save_waypoint = rospy.ServiceProxy("bw_waypoints/save_waypoint", SaveWaypoint)
        self.save_tf = rospy.ServiceProxy("bw_waypoints/save_tf", SaveTF)
        self.tags_request = rospy.ServiceProxy("tags_request", RequestTags)

        self.teach_waypoint_srv = rospy.Service("teach_waypoint", TeachWaypoint, self.teach_waypoint)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

    def get_offset(self, name: str) -> Pose:
        return self.named_offsets.get(name).to_ros_pose()

    def teach_waypoint(self, req: TeachWaypointRequest) -> TeachWaypointResponse:
        response = TeachWaypointResponse()
        if self.tag_manager.is_name_tag(req.name):
            rospy.loginfo("Saving waypoint %s from tag" % req.name)
            response.success = self.request_waypoint_from_tag(req.name, self.get_offset(req.name))
        else:
            rospy.loginfo("Saving waypoint %s from robot's pose" % req.name)
            response.success = self.save_waypoint_at_robot(req.name)
        rospy.loginfo("Saving waypoint was %s" % ("successful" if response.success else "unsuccessful"))
        return response

    def request_waypoint_from_tag(self, name: str, offset: Pose) -> bool:
        tags = self.tags_request()
        tag_result = self.tag_manager.get_tag_by_name(name, tags.tags)
        if tag_result is None:
            return False

        tag_camera_relative = PoseStamped()
        tag_camera_relative.header = tag_result.detection.pose.header
        tag_camera_relative.pose = tag_result.detection.pose.pose.pose
        tag_global = lookup_pose_in_frame(self.tf_buffer, tag_camera_relative, self.map_frame)
        if tag_global is None:
            return False
        tag_global.pose.orientation = TagManager.rotate_tag(tag_global.pose.orientation)
        tag_offset = transform_pose(offset, tag_global)

        waypoint = Waypoint()
        waypoint.header = tag_offset.header
        waypoint.name = name
        waypoint.pose = tag_offset.pose

        response = self.save_waypoint(waypoint)
        return response.success

    def save_waypoint_at_robot(self, name: str) -> bool:
        response = self.save_tf(name, self.map_frame, self.base_frame)
        return response.success

    def run(self) -> None:
        rospy.spin()


if __name__ == "__main__":
    node = WaypointTeacher()
    node.run()
