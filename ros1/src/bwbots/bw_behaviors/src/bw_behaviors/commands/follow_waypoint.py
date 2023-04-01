import rospy
import actionlib

from actionlib_msgs.msg import GoalStatus

from bw_interfaces.msg import (
    FollowWaypointsAction,
    FollowWaypointsGoal,
    FollowWaypointsFeedback,
    FollowWaypointsResult,
)
from bw_interfaces.msg import Waypoint

from geometry_msgs.msg import PoseStamped

from simple_move_base_client import SimpleMoveBaseClient


class FollowWaypoint:
    def __init__(self, move_base_client: SimpleMoveBaseClient) -> None:
        self.move_base_client = move_base_client

        self.current_waypoint = Waypoint()
        self.action_server = actionlib.SimpleActionServer(
            "follow_waypoints",
            FollowWaypointsAction,
            execute_cb=self.action_callback,
            auto_start=False,
        )
        self.action_server.start()
        rospy.loginfo("follow_waypoints is ready")

    def action_callback(self, goal: FollowWaypointsGoal):
        rospy.loginfo(f"Following waypoints: {goal}")
        self.move_base_client.connect()

        result = FollowWaypointsResult(True)
        for waypoint in goal.waypoints.waypoints:
            mb_goal = PoseStamped()
            mb_goal.header.frame_id = waypoint.header.frame_id
            mb_goal.pose = waypoint.pose
            self.move_base_client.send_goal(mb_goal, self.move_base_feedback)
            wait_result = self.move_base_client.wait(
                lambda: self.action_server.is_preempt_requested()
            )
            if (
                self.action_server.is_preempt_requested()
                or wait_result != GoalStatus.SUCCEEDED
                or not self.move_base_client.did_succeed()
            ):
                rospy.loginfo(
                    f"Failed to get to waypoint. is preempt requested: {self.action_server.is_preempt_requested()}. "
                    f"Move base wait result: {wait_result}. "
                    f"Move base state: {self.move_base_client.get_state()}"
                )
                result = FollowWaypointsResult(False)
                self.action_server.set_aborted(
                    result, "Interrupted while going to a waypoint"
                )
                return
        self.action_server.set_succeeded(result)

    def move_base_feedback(self, mb_feedback):
        feedback = FollowWaypointsFeedback()
        feedback.current_pose = mb_feedback.base_position
        feedback.current_goal = self.current_waypoint
        self.action_server.publish_feedback(feedback)
