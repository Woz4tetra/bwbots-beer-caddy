#!/usr/bin/env python3
import rospy
import actionlib

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist

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
        pose_goal.timeout = rospy.Duration(kwargs.get("timeout", 2.0))
        pose_goal.reference_linear_speed = kwargs.get("reference_linear_speed", 0.5)
        pose_goal.reference_angular_speed = kwargs.get("reference_angular_speed", 3.0)
        pose_goal.allow_reverse = kwargs.get("allow_reverse", True)
        pose_goal.rotate_in_place_start = kwargs.get("rotate_in_place_start", True)
        pose_goal.rotate_in_place_end = kwargs.get("rotate_in_place_end", True)
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
    cmd_vel_pub = rospy.Publisher("/bw/cmd_vel", Twist, queue_size=10)
    
    find_tag_action = actionlib.SimpleActionClient("/bw/find_tag", FindTagAction)
    go_to_pose_action = actionlib.SimpleActionClient("/bw/go_to_pose", GoToPoseAction)
    rospy.loginfo("Connecting to action server...")
    find_tag_action.wait_for_server()
    go_to_pose_action.wait_for_server()

    tag_goal = FindTagGoal()
    tag_goal.tag_id = []
    tag_goal.reference_frame_id = "odom"
    
    point_forward_twist = Twist()
    point_forward_twist.linear.x = 0.02

    try:
        rospy.sleep(1.5)

        find_tag_action.send_goal(tag_goal, done_cb=tag_action_done, feedback_cb=feedback_cb)
        find_tag_action.wait_for_result()

        if not go_to_tag(-0.45, xy_tolerance=0.1, yaw_tolerance=0.01, timeout=10.0, reference_linear_speed=0.35, reference_angular_speed=3.0, allow_reverse=True):
            return
    
        for _ in range(10):
            cmd_vel_pub.publish(point_forward_twist)
            rospy.sleep(0.005)
        cmd_vel_pub.publish(Twist())

        rospy.sleep(1.5)

        find_tag_action.send_goal(tag_goal, done_cb=tag_action_done, feedback_cb=feedback_cb)
        find_tag_action.wait_for_result()

        if not go_to_tag(-0.35, xy_tolerance=0.02, yaw_tolerance=0.03, reference_linear_speed=0.35, reference_angular_speed=3.0, allow_reverse=False, rotate_in_place_start=False, rotate_in_place_end=False):
            return

        if not go_to_tag(-0.075, xy_tolerance=0.05, yaw_tolerance=0.5, reference_linear_speed=0.5, reference_angular_speed=0.01, allow_reverse=False, rotate_in_place_start=False, rotate_in_place_end=False):
            return

    except KeyboardInterrupt:
        rospy.loginfo("Cancelling goal")
        find_tag_action.cancel_goal()
        go_to_pose_action.cancel_goal()
        rospy.sleep(1.0)


if __name__ == '__main__':
    main()
