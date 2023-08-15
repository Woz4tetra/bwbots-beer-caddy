from typing import Optional
import rospy
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import TransformStamped, PoseStamped


def lookup_transform(
    tf_buffer, parent_link, child_link, time_window=None, timeout=None, silent=False
) -> Optional[TransformStamped]:
    """
    Call tf_buffer.lookup_transform. Return None if the look up fails
    """
    if time_window is None:
        time_window = rospy.Time(0)
    else:
        time_window = rospy.Time.now() - time_window

    if timeout is None:
        timeout = rospy.Duration(1.0)  # type: ignore

    try:
        return tf_buffer.lookup_transform(parent_link, child_link, time_window, timeout)
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException, tf2_ros.InvalidArgumentException) as e:  # type: ignore
        if not silent:
            rospy.logwarn(
                "Failed to look up %s to %s. %s" % (parent_link, child_link, e)
            )
        return None


def transform_pose(
    tf_buffer,
    pose_stamped: PoseStamped,
    destination_frame: str,
    time_window=None,
    timeout=None,
    silent=False,
) -> Optional[PoseStamped]:
    transform = lookup_transform(
        tf_buffer,
        destination_frame,
        pose_stamped.header.frame_id,
        time_window,
        timeout,
        silent,
    )
    if transform is None:
        return None
    return tf2_geometry_msgs.do_transform_pose(pose_stamped, transform)
