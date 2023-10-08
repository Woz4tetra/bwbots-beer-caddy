from threading import Lock

import actionlib
import rospy
from py_trees.common import Status

from bw_interfaces.msg import GoToPoseAction, GoToPoseFeedback, GoToPoseGoal, GoToPoseResult


class GoToManager:
    def __init__(self):
        self.action = actionlib.SimpleActionClient("go_to_pose", GoToPoseAction)
        self.action.wait_for_server()
        self.status = Status.FAILURE
        self.lock = Lock()
        rospy.loginfo("Simple go to pose action connected")

    def send_goal(self, goal: GoToPoseGoal) -> None:
        with self.lock:
            self.status = Status.RUNNING
        self.action.send_goal(goal, done_cb=self.action_done, feedback_cb=self.feedback_cb)

    def get_status(self) -> Status:
        with self.lock:
            return self.status

    def action_done(self, goal_status, result: GoToPoseResult) -> None:
        with self.lock:
            self.status = Status.SUCCESS if result.success else Status.FAILURE
        rospy.loginfo(f"Action finished with result: {result.success}. Status: {goal_status}")

    def feedback_cb(self, feedback: GoToPoseFeedback) -> None:
        pass

    def cancel(self) -> None:
        self.action.cancel_all_goals()
