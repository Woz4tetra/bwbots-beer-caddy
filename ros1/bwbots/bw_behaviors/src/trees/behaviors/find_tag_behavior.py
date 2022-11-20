from typing import List, Optional
import rospy
import py_trees
import py_trees_ros

from geometry_msgs.msg import PoseStamped

from bw_interfaces.msg import FindTagAction, FindTagGoal, FindTagResult

from trees.managers.tag_manager import TagManager


class FindTagBehavior(py_trees_ros.actions.ActionClient):
    def __init__(self, tag_name: str, tag_manager: TagManager, timeout: float):
        self.tag_manager = tag_manager
        self.tag_name = tag_name
        tag = self.tag_manager.get_tag(self.tag_name)
        goal = FindTagGoal()
        goal.tag_id = tag.tag_id
        goal.reference_frame_id = tag.reference_frame
        goal.timeout = rospy.Duration(timeout)

        self.tag_result_pub: Optional[rospy.Publisher] = None

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
            if self.tag_manager.is_tag_valid(self.tag_name):
                return py_trees.Status.FAILURE
            
            result: FindTagResult = self.action_client.get_result()
            self.tag_result_pub.publish(result.pose)
            self.tag_manager.set_tag(self.tag_name, result.pose)

            rospy.loginfo(f"{self.tag_name} found: {result.success}")
        else:
            self.tag_manager.unset_tag(self.tag_name)
        return action_result
