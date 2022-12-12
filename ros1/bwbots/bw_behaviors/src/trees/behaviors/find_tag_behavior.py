from typing import Callable
import rospy
import py_trees
import py_trees_ros

from geometry_msgs.msg import PoseStamped

from bw_interfaces.msg import FindTagAction, FindTagGoal, FindTagResult

from trees.managers.tag_manager import TagManager


class FindTagBehavior(py_trees_ros.actions.ActionClient):
    def __init__(self, tag_name_supplier: Callable[[], str], tag_manager: TagManager, timeout: float):
        self.tag_manager = tag_manager
        self.tag_name_supplier = tag_name_supplier
        self.tag_name = None
        self.tag_result_pub = rospy.Publisher("/bw/tag_pose", PoseStamped, queue_size=10)

        super().__init__("Find tag",
            FindTagAction,
            action_namespace="/bw/find_tag")
    
        self.action_goal = FindTagGoal()
        self.action_goal.timeout = rospy.Duration(timeout)

    def update(self):
        if not self.sent_goal:
            self.tag_name = self.tag_name_supplier()
            if type(self.tag_name) != str:
                return py_trees.Status.FAILURE
            tag = self.tag_manager.get_tag(self.tag_name)
            self.action_goal.tag_id = tag.tag_id
            self.action_goal.reference_frame_id = tag.reference_frame

        action_result = super().update()
        if action_result == py_trees.Status.SUCCESS:
            
            result: FindTagResult = self.action_client.get_result()
            self.tag_result_pub.publish(result.pose)
            self.tag_manager.set_tag(self.tag_name, result.pose)

            rospy.loginfo(f"{self.tag_name} found: {result.success}")
            if result.success and self.tag_manager.is_tag_valid(self.tag_name):
                return py_trees.Status.SUCCESS
            else:
                return py_trees.Status.FAILURE
        else:
            self.tag_manager.unset_tag(self.tag_name)
        return action_result
