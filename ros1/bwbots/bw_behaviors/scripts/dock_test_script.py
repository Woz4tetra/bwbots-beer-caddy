#!/usr/bin/env python3
import rospy
import actionlib

from geometry_msgs.msg import PoseStamped

from bw_interfaces.msg import FindTagAction, FindTagGoal, FindTagFeedback, FindTagResult
from bw_interfaces.msg import GoToPoseAction, GoToPoseGoal, GoToPoseResult
from bw_tools.robot_state import Pose2d


def main():
    tag_pose = Pose2d()
    pose_result = GoToPoseResult()

    def pose_action_done(goal_status, result: GoToPoseResult):
        nonlocal pose_result
        pose_result = result

    def tag_action_done(goal_status, result: FindTagResult):
        nonlocal tag_pose
        rospy.loginfo(f"Action finished with result: {result.success}. Status: {goal_status}")
        tag_pose = Pose2d.from_ros_pose(result.pose.pose)
        tag_result_pub.publish(result.pose)
        rospy.loginfo(f"Tag header: {result.pose.header.frame_id}")
        rospy.loginfo(f"Tag location: {{x: {tag_pose.x}, y: {tag_pose.y}, theta: {tag_pose.theta}}}")

    def feedback_cb(feedback: FindTagFeedback):
        tag_sample_pub.publish(feedback.sample)
    
    def go_to_tag(distance_offset, **kwargs):
        offset = Pose2d(0.0, distance_offset, 1.5708)
        pose2d = offset.transform_by(tag_pose)

        pose_goal = GoToPoseGoal()
        pose_goal.goal.pose = pose2d.to_ros_pose()
        pose_goal.goal.header.frame_id = tag_goal.reference_frame_id
        pose_goal.xy_tolerance = kwargs.get("xy_tolerance", 0.05)
        pose_goal.yaw_tolerance = kwargs.get("yaw_tolerance", 0.15)
        pose_goal.timeout = kwargs.get("timeout", rospy.Duration(2.0))
        pose_goal.reference_linear_speed = kwargs.get("reference_linear_speed", 0.5)
        pose_goal.reference_angular_speed = kwargs.get("reference_angular_speed", 3.0)
        pose_goal.allow_reverse = kwargs.get("allow_reverse", True)
        go_to_pose_action.send_goal(pose_goal, done_cb=pose_action_done)
        go_to_pose_action.wait_for_result()
        rospy.loginfo(f"Go to pose result: {pose_result.success}")
        return pose_result.success

    rospy.init_node(
        "go_to_pose_action_script",
        disable_signals=True,
        # log_level=rospy.DEBUG
    )

    tag_sample_pub = rospy.Publisher("/bw/tag_sample", PoseStamped, queue_size=10)
    tag_result_pub = rospy.Publisher("/bw/tag_pose", PoseStamped, queue_size=10)
    
    find_tag_action = actionlib.SimpleActionClient("/bw/find_tag", FindTagAction)
    go_to_pose_action = actionlib.SimpleActionClient("/bw/go_to_pose", GoToPoseAction)
    rospy.loginfo("Connecting to action server...")
    find_tag_action.wait_for_server()
    go_to_pose_action.wait_for_server()

    tag_goal = FindTagGoal()
    tag_goal.tag_id = []
    tag_goal.reference_frame_id = "odom"

    try:
        rospy.sleep(1.5)

        find_tag_action.send_goal(tag_goal, done_cb=tag_action_done, feedback_cb=feedback_cb)
        find_tag_action.wait_for_result()

        if not go_to_tag(-0.5, xy_tolerance=0.05, yaw_tolerance=0.15, reference_linear_speed=0.5, reference_angular_speed=3.0, allow_reverse=True):
            return

        rospy.sleep(1.5)

        find_tag_action.send_goal(tag_goal, done_cb=tag_action_done, feedback_cb=feedback_cb)
        find_tag_action.wait_for_result()

        if not go_to_tag(-0.35, xy_tolerance=0.03, yaw_tolerance=0.1, reference_linear_speed=0.4, reference_angular_speed=0.8, allow_reverse=False):
            return

        if not go_to_tag(-0.1, xy_tolerance=0.1, yaw_tolerance=0.25, reference_linear_speed=0.5, reference_angular_speed=0.1, allow_reverse=False):
            return

    except KeyboardInterrupt:
        rospy.loginfo("Cancelling goal")
        find_tag_action.cancel_goal()
        go_to_pose_action.cancel_goal()
        rospy.sleep(1.0)


if __name__ == '__main__':
    main()
