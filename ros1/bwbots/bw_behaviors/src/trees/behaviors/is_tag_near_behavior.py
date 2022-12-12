from typing import Callable, Optional
import tf2_ros
import py_trees
import tf2_geometry_msgs

from geometry_msgs.msg import PoseStamped

from bw_tools.robot_state import Pose2d

from trees.managers.tag_manager import TagManager
from trees.managers.tag_manager import Tag


class IsTagNearBehavior(py_trees.behaviour.Behaviour):
    def __init__(self, robot_frame: str, tag_name_supplier: Callable[[], str], tag_manager: TagManager, distance_threshold: float):
        self.distance_threshold = distance_threshold
        self.robot_frame = robot_frame
        self.tag_name_supplier = tag_name_supplier
        self.tag_manager = tag_manager
        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        super().__init__("Is tag near")

    def tf_tag(self, tag: Tag) -> Optional[Pose2d]:
        try:
            transform = self.tf_buffer.lookup_transform(self.robot_frame, tag.reference_frame, rospy.Time(0), rospy.Duration(1.0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            return None

        tag_pose = PoseStamped()
        tag_pose.header.frame_id = tag.reference_frame
        tag_pose.pose = tag.pose
        dest_tag_pose = tf2_geometry_msgs.do_transform_pose(tag_pose, transform)
        return Pose2d.from_ros_pose(dest_tag_pose.pose)

    def update(self):
        tag_name = self.tag_name_supplier()
        if type(tag_name) != str:
            return py_trees.Status.FAILURE
        
        if self.tag_manager.is_tag_valid(tag_name):
            tag = self.tag_manager.get_tag(tag_name)
            tag_pose = self.tf_tag(tag)
            if tag_pose is None:
                return py_trees.Status.FAILURE
            if tag_pose.distance() < self.distance_threshold:
                return py_trees.Status.SUCCESS
            else:
                return py_trees.Status.FAILURE
        else:
            return py_trees.Status.FAILURE
