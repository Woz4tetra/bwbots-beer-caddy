import rospy
import py_trees_ros
from bw_interfaces.msg import FollowDetectionAction, FollowDetectionGoal


class FollowDetectionBehavior(py_trees_ros.actions.ActionClient):
    def __init__(self, label: str, offset_distance: float, detection_timeout: float):
        action_goal = FollowDetectionGoal()
        action_goal.class_name = label
        action_goal.offset_distance = offset_distance
        action_goal.detection_timeout = rospy.Duration(
            detection_timeout  # type: ignore
        )

        super().__init__(
            "Follow Waypoint",
            FollowDetectionAction,
            action_goal,
            action_namespace="/bw/follow_detection",
        )
