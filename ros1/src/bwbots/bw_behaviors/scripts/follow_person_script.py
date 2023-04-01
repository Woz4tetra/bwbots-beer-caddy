#!/usr/bin/env python3
import rospy
import actionlib
import argparse

from geometry_msgs.msg import PoseStamped

from bw_interfaces.msg import (
    FollowDetectionAction,
    FollowDetectionGoal,
    FollowDetectionFeedback,
    FollowDetectionResult,
)


def main():
    def action_done(goal_status, result: FollowDetectionResult):
        rospy.loginfo(f"Action finished with result: {result}")

    def feedback_cb(feedback: FollowDetectionFeedback):
        goal_pub.publish(feedback.chase_pose)
        rospy.loginfo(f"Distance from detection: {feedback.distance_from_detection}")

    rospy.init_node(
        "follow_detection_action_script",
        disable_signals=True,
        # log_level=rospy.DEBUG
    )

    action = actionlib.SimpleActionClient("/bw/follow_detection", FollowDetectionAction)
    rospy.loginfo("Connecting to action server...")
    action.wait_for_server()

    goal_pub = rospy.Publisher("/bw/go_to_pose_goal", PoseStamped, queue_size=10)

    parser = argparse.ArgumentParser(description="action_script")

    parser.add_argument(
        "-l", "--label", type=str, default="person", help="Label of detection to follow"
    )
    parser.add_argument(
        "-o", "--offset", type=float, default=0.6, help="Distance to offset from goal"
    )
    args = parser.parse_args()

    goal = FollowDetectionGoal()
    goal.class_name = args.label
    goal.offset_distance = args.offset
    goal.detection_timeout = rospy.Duration(30.0)  # type: ignore

    rospy.loginfo(f"Following {args.label}")
    action.send_goal(goal, done_cb=action_done, feedback_cb=feedback_cb)
    try:
        action.wait_for_result()
    except KeyboardInterrupt:
        rospy.loginfo("Cancelling goal")
        action.cancel_goal()
        rospy.sleep(1.0)


if __name__ == "__main__":
    main()
