import rospy
from typing import Callable
import py_trees
import py_trees_ros

from bw_interfaces.msg import DispenseAction, DispenseGoal


class SendDispenseCommandBehavior(py_trees_ros.actions.ActionClient):
    def __init__(self, dispenser_name_supplier: Callable[[], str], timeout: float = 5.0):
        super().__init__("Send Dispense Command",
            DispenseAction,
            action_namespace="/bw/dispense")
        self.action_timeout = rospy.Duration(timeout)
        self.dispenser_name_supplier = dispenser_name_supplier

    def update(self):
        if not self.sent_goal:
            dispenser_name = self.dispenser_name_supplier()
            if type(dispenser_name) != str:
                rospy.logwarn(f"Supplied dispenser name is not a string! {dispenser_name}")
                return py_trees.Status.FAILURE
            
            rospy.loginfo(f"Requesting {dispenser_name} to dispense")
            self.action_goal = DispenseGoal()
            self.action_goal.dispenser_name = dispenser_name
            self.action_goal.timeout = self.action_timeout
        return super().update()
