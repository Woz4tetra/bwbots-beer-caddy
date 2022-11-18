from typing import List, Optional
import rospy
import py_trees
import py_trees_ros

from geometry_msgs.msg import PoseStamped

from bw_interfaces.msg import FindTagAction, FindTagGoal, FindTagResult


class FindTagBehavior(py_trees_ros.actions.ActionClient):
    def __init__(self, tag_id: List[int], reference_frame_id, blackboard_name):
        self.blackboard_name = blackboard_name
        goal = FindTagGoal()
        goal.tag_id = tag_id
        goal.reference_frame_id = reference_frame_id

        self.tag_result_pub: Optional[rospy.Publisher] = None
        self.blackboard = py_trees.blackboard.Blackboard()

        super().__init__("Find tag",
            FindTagAction,
            goal,
            action_namespace="/bw/find_tag")

    def setup(self, timeout):
        self.tag_result_pub = rospy.Publisher("/bw/tag_pose", PoseStamped, queue_size=10)
        return super().setup(timeout)

    def update(self):
        action_result = super().update()
        if action_result == py_trees.Status.SUCCESS:
            result: FindTagResult = self.action_client.get_result()
            self.tag_result_pub.publish(result.pose)
            
            rospy.loginfo(f"{self.blackboard_name} found: {result.success}")
            self.blackboard.set(self.blackboard_name, result.pose)
        return action_result
