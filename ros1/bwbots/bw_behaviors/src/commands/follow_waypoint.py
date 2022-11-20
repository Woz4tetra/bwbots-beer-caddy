import rospy
import actionlib

from actionlib_msgs.msg import GoalStatus

from bw_interfaces.msg import FollowWaypointsAction, FollowWaypointsGoal, FollowWaypointsFeedback, FollowWaypointsResult
from bw_interfaces.msg import Waypoint

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseResult


class FollowWaypoint:
    def __init__(self) -> None:
        self.move_base_namespace = rospy.get_param("~follow_pose/move_base_namespace", "/move_base")
        self.move_base = actionlib.SimpleActionClient(self.move_base_namespace, MoveBaseAction)

        self.current_waypoint = Waypoint()
        self.is_move_base_done = False
        self.move_base_succeeded = False
        self.action_server = actionlib.SimpleActionServer(
            "follow_waypoints",
            FollowWaypointsAction,
            execute_cb=self.action_callback,
            auto_start=False
        )
        self.action_server.start()
        rospy.loginfo("follow_waypoints is ready")

    def wait_for_move_base(self):
        rate = rospy.Rate(10.0)
        status = GoalStatus.PENDING
        while not self.is_move_base_done:
            if rospy.is_shutdown():
                rospy.loginfo("Received abort. Cancelling waypoint goal")
                status = GoalStatus.ABORTED
                self.move_base.cancel_goal()
                break

            if self.action_server.is_preempt_requested():
                rospy.loginfo("Received preempt. Cancelling waypoint goal")
                status = GoalStatus.ABORTED
                self.move_base.cancel_goal()
                break
            status = GoalStatus.ACTIVE
            rate.sleep()
        status = GoalStatus.SUCCEEDED
        return status

    def action_callback(self, goal: FollowWaypointsGoal):
        rospy.loginfo(f"Following waypoints: {goal}")

        rospy.loginfo("Connecting to move_base...")
        self.move_base.wait_for_server()
        rospy.loginfo("move_base connected")
        
        result = FollowWaypointsResult(True)
        for waypoint in goal.waypoints.waypoints:
            mb_goal = MoveBaseGoal()
            mb_goal.target_pose.header.frame_id = waypoint.header.frame_id
            mb_goal.target_pose.pose = waypoint.pose
            self.current_waypoint = waypoint
            self.is_move_base_done = False
            self.move_base_succeeded = False
            self.move_base.send_goal(mb_goal, feedback_cb=lambda feedback: self.move_base_feedback(feedback), done_cb=self.move_base_done)
            wait_result = self.wait_for_move_base()
            mb_state = self.move_base.get_state()
            if self.action_server.is_preempt_requested() or \
                    wait_result != GoalStatus.SUCCEEDED or \
                    not self.move_base_succeeded:
                rospy.loginfo(
                    f"Failed to get to waypoint. is preempt requested: {self.action_server.is_preempt_requested()}. "
                    f"Move base wait result: {wait_result}. "
                    f"Move base state: {mb_state}"
                )
                result = FollowWaypointsResult(False)
                self.action_server.set_aborted(result, "Interrupted while going to a waypoint")
                return
        self.action_server.set_succeeded(result)

    def move_base_feedback(self, mb_feedback):
        feedback = FollowWaypointsFeedback()
        feedback.current_pose = mb_feedback.base_position
        feedback.current_goal = self.current_waypoint
        self.action_server.publish_feedback(feedback)
        
    def move_base_done(self, goal_status: GoalStatus, result: MoveBaseResult):
        rospy.loginfo("move_base finished: %s. %s" % (goal_status, result))
        self.is_move_base_done = True
        self.move_base_succeeded = goal_status == GoalStatus.SUCCEEDED
