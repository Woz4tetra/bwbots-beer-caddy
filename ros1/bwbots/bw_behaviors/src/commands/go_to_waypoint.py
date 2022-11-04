import rospy
import actionlib

from actionlib_msgs.msg import GoalStatus

from bw_interfaces.msg import FollowWaypointsAction, FollowWaypointsGoal, FollowWaypointsFeedback, FollowWaypointsResult
from bw_interfaces.msg import Waypoint

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseResult


class GoToWaypointCommand:
    def __init__(self) -> None:
        self.move_base_namespace = rospy.get_param("~go_to_waypoint/move_base_namespace", "/move_base")
        self.move_base = actionlib.SimpleActionClient(self.move_base_namespace, MoveBaseAction)

        self.goal = actionlib.ServerGoalHandle()

        rospy.loginfo("Connecting to move_base...")
        self.move_base.wait_for_server()
        rospy.loginfo("move_base connected")
        
        self.current_waypoint = Waypoint()
        self.is_move_base_done = False
        self.is_cancelled = False
        self.follow_waypoints_server = actionlib.ActionServer(
            "follow_waypoints",
            FollowWaypointsAction,
            self.follow_waypoints_callback, 
            auto_start=False
        )
        self.follow_waypoints_server.start()
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

            if self.is_cancelled:
                rospy.loginfo("Received preempt. Cancelling waypoint goal")
                status = GoalStatus.ABORTED
                self.move_base.cancel_goal()
                break
            status = GoalStatus.ACTIVE
            rate.sleep()
        status = GoalStatus.SUCCEEDED
        return status

    def cancel_callback(self, goal_handle: actionlib.ServerGoalHandle):
        self.is_cancelled = True

    def follow_waypoints_callback(self, goal_handle: actionlib.ServerGoalHandle):
        if self.goal.get_goal() and \
				self.goal.get_goal_status().status == GoalStatus.ACTIVE:
            rospy.loginfo("Cancelling current goal")
            self.move_base.cancel_goal()
        self.goal = goal_handle
        goal: FollowWaypointsGoal = goal_handle.get_goal()
        self.goal.set_accepted("Waypoints accepted")
        self.is_cancelled = False

        result = FollowWaypointsResult(True)
        for waypoint in goal.waypoints.waypoints:
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = waypoint.header.frame_id
            goal.target_pose.pose = waypoint.pose
            self.current_waypoint = waypoint
            self.is_move_base_done = False
            self.move_base.send_goal(goal, feedback_cb=self.move_base_feedback, done_cb=self.move_base_done)
            wait_result = self.wait_for_move_base()
            if self.is_cancelled or wait_result != GoalStatus.SUCCEEDED or not self.move_base.get_result():
                result = FollowWaypointsResult(False)
                break
        if result.success:
            self.goal.set_succeeded(result)
        else:
            self.goal.set_aborted(result, "Failed to get to a waypoint")

    def move_base_feedback(self, mb_feedback):
        feedback = FollowWaypointsFeedback()
        feedback.current_pose = mb_feedback.base_position
        feedback.current_goal = self.current_waypoint
        self.goal.publish_feedback(feedback)
        
    def move_base_done(self, goal_status: GoalStatus, result: MoveBaseResult):
        rospy.loginfo("move_base finished: %s. %s" % (goal_status, result))
        self.is_move_base_done = True
