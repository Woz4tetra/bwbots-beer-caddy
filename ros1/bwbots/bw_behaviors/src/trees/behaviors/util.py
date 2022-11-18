import math

from bw_tools.robot_state import Pose2d

from geometry_msgs.msg import PoseStamped


def get_offset_tag(dock_tag_pose_stamped: PoseStamped, x_offset, y_offset) -> PoseStamped:
    dock_tag_pose2d = Pose2d.from_ros_pose(dock_tag_pose_stamped.pose)
    offset = Pose2d(y_offset, x_offset, math.pi / 2.0)
    rotate_dock_tag_pose2d = offset.transform_by(dock_tag_pose2d)
    rotate_dock_tag_pose_stamped = PoseStamped()
    rotate_dock_tag_pose_stamped.header = dock_tag_pose_stamped.header
    rotate_dock_tag_pose_stamped.pose = rotate_dock_tag_pose2d.to_ros_pose()
    return rotate_dock_tag_pose_stamped
