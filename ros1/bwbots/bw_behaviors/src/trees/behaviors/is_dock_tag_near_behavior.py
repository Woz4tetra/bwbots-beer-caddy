from typing import List
import rospy
import py_trees
import py_trees_ros

from bw_interfaces.msg import FindTagAction, FindTagGoal, FindTagResult

from bw_tools.robot_state import Pose2d


class IsDockTagNearBehavior(py_trees_ros.actions.ActionClient):
    def __init__(self, tag_id: List[int], reference_frame_id, distance_threshold: float):
        self.distance_threshold = distance_threshold

        goal = FindTagGoal()
        goal.tag_id = tag_id
        goal.reference_frame_id = reference_frame_id

        super().__init__("Is dock tag near",
            FindTagAction,
            goal,
            action_namespace="/bw/find_tag")

    def update(self):
        action_result = super().update()
        if action_result == py_trees.Status.SUCCESS:
            result: FindTagResult = self.action_client.get_result()
            tag_pose2d = Pose2d.from_ros_pose(result.pose.pose)
            tag_distance = tag_pose2d.distance()
            rospy.loginfo(f"Dock tag found: {result.success}. Tag distance: {tag_distance}")
            if tag_distance < self.distance_threshold:
                return py_trees.Status.SUCCESS
            else:
                return py_trees.Status.FAILURE
        elif action_result == py_trees.Status.RUNNING:
            return py_trees.Status.RUNNING
        else:
            return py_trees.Status.FAILURE
