from typing import List, Optional
import rospy
import py_trees
import py_trees_ros

from geometry_msgs.msg import PoseStamped

from bw_interfaces.msg import SetRobotStateAction, SetRobotStateGoal, SetRobotStateResult


class SetRobotStateBehavior(py_trees_ros.actions.ActionClient):
    def __init__(self, motors_enabled, timeout=1.0):
        goal = SetRobotStateGoal()
        goal.enabled = motors_enabled
        goal.timeout = rospy.Duration(timeout)

        super().__init__("Set Robot State",
            SetRobotStateAction,
            goal,
            action_namespace="/bw/set_robot_state")

    def update(self):
        action_result = super().update()
        if action_result == py_trees.Status.SUCCESS:
            result: SetRobotStateResult = self.action_client.get_result()
            if result.success:
                return py_trees.Status.SUCCESS
            else:
                return py_trees.Status.FAILURE
        return action_result
