import math
import genpy
import rospy
from dataclasses import dataclass
from typing import Dict, Tuple, Optional, Union
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from bw_tools.robot_state import Pose2d


@dataclass
class Tag:
    name: str
    tag_id: Tuple[int]
    reference_frame: str
    pose: Optional[Pose] = None
    timestamp: Optional[genpy.Time] = None
    prep: str = ""
    tag_type: str = ""


class TagManager:
    def __init__(self, valid_tag_window: float) -> None:
        self.tags: Dict[str, Tag] = {}
        self.metadata = {}
        self.valid_tag_window = rospy.Duration(valid_tag_window)  # type: ignore

    def register_tag(self, tag: Tag) -> None:
        self.tags[tag.name] = tag

    def set_tag(self, name: str, pose_stamped: Optional[PoseStamped]) -> None:
        if pose_stamped is None:
            self.tags[name].pose = None
            self.tags[name].timestamp = None
        else:
            self.tags[name].pose = pose_stamped.pose
            self.tags[name].timestamp = pose_stamped.header.stamp
            if len(pose_stamped.header.frame_id) != 0:
                self.tags[name].reference_frame = pose_stamped.header.frame_id
            else:
                rospy.logwarn(
                    f"The reference frame for tag {name} is not valid! pose_stamped: {pose_stamped}"
                )

    def unset_tag(self, name: str) -> None:
        self.set_tag(name, None)

    def is_tag_valid(
        self, name, valid_tag_window: Optional[rospy.Duration] = None
    ) -> bool:
        if self.tags[name].timestamp is None or self.tags[name].pose is None:
            return False
        if valid_tag_window is None:
            valid_tag_window = self.valid_tag_window
        if valid_tag_window == rospy.Duration(0):
            return True
        return rospy.Time.now() - self.tags[name].timestamp < valid_tag_window

    def get_tag(self, name) -> Tag:
        return self.tags[name]

    def get_offset_tag(
        self,
        name: str,
        x_offset: float,
        y_offset: float,
        theta_offset: float,
        valid_tag_window: Optional[rospy.Duration] = None,
    ) -> Optional[PoseStamped]:
        if not self.is_tag_valid(name, valid_tag_window):
            return None
        tag = self.get_tag(name)
        tag_pose2d = Pose2d.from_ros_pose(tag.pose)
        offset = Pose2d(x_offset, y_offset, theta_offset)
        rotate_tag_pose2d = offset.transform_by(tag_pose2d)
        rotate_tag_pose_stamped = PoseStamped()
        rotate_tag_pose_stamped.header.frame_id = tag.reference_frame
        rotate_tag_pose_stamped.pose = rotate_tag_pose2d.to_ros_pose()
        return rotate_tag_pose_stamped
