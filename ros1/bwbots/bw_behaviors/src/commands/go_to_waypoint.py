import rospy
import actionlib

from actionlib_msgs.msg import GoalStatus

from bw_interfaces.msg import FollowWaypointsAction, FollowWaypointsGoal, FollowWaypointsFeedback, FollowWaypointsResult
from bw_interfaces.msg import Waypoint

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseResult


class GoToWaypointCommand:
    def __init__(self) -> None:
        self.move_base = actionlib.SimpleActionClient(self.move_base_namespace, MoveBaseAction)

        rospy.loginfo("Connecting to move_base...")
        self.move_base.wait_for_server()
        rospy.loginfo("move_base connected")
        
        self.current_waypoint = Waypoint()
        self.is_move_base_done = False
        self.follow_waypoints_server = actionlib.ActionServer(
            "follow_waypoints",
            FollowWaypointsAction,
            self.follow_waypoints_callback, 
            auto_start=False
        )
        self.follow_waypoints_server.start()

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

    def follow_waypoints_callback(self, goal: FollowWaypointsGoal):
        result = FollowWaypointsResult(True)
        for waypoint in goal.waypoints.waypoints:
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = waypoint.header.frame_id
            goal.target_pose = waypoint.pose
            self.current_waypoint = waypoint
            self.is_move_base_done = False
            self.move_base.send_goal(goal, feedback_cb=self.move_base_feedback, done_cb=self.move_base_done)
            wait_result = self.wait_for_move_base()
            if wait_result != GoalStatus.SUCCEEDED or not self.move_base.get_result():
                result = FollowWaypointsResult(False)
                break
        self.follow_waypoints_server.publish_result(result)
        if result.success:
            self.follow_waypoints_server.set_succeeded()
        else:
            self.follow_waypoints_server.set_aborted()

    def move_base_feedback(self, mb_feedback):
        feedback = FollowWaypointsFeedback()
        feedback.current_pose = mb_feedback.base_position
        feedback.current_waypoint = self.current_waypoint
        self.follow_waypoints_server.publish_feedback(feedback)
        
    def move_base_done(self, goal_status: GoalStatus, result: MoveBaseResult):
        rospy.loginfo("move_base finished: %s. %s" % (goal_status, result))
        self.is_move_base_done = True
