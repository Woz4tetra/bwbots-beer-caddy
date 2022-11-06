#!/usr/bin/env python3
import rospy
import actionlib
import argparse

from geometry_msgs.msg import PoseStamped

from bw_interfaces.msg import FindTagAction, FindTagGoal, FindTagFeedback, FindTagResult


def main():
    def action_done(goal_status, result: FindTagResult):
        rospy.loginfo(f"Action finished with result: {result}. Status: {goal_status}")
        tag_sample_pub.publish(result.pose)


    def feedback_cb(feedback: FindTagFeedback):
        tag_sample_pub.publish(feedback.sample)
        
    rospy.init_node(
        "go_to_pose_action_script",
        disable_signals=True,
        # log_level=rospy.DEBUG
    )

    tag_sample_pub = rospy.Publisher("/bw/tag_pose", PoseStamped, queue_size=10)
    
    action = actionlib.SimpleActionClient("/bw/find_tag", FindTagAction)
    rospy.loginfo("Connecting to action server...")
    action.wait_for_server()

    parser = argparse.ArgumentParser(description="action_script")

    parser.add_argument("-t", "--tag-id",
                        default=[],
                        help="Tag to search for")
    parser.add_argument("-r", "--reference-frame",
                        default="map",
                        help="Frame to publish tags in")
    args = parser.parse_args()

    goal = FindTagGoal()
    goal.tag_id = args.tag_id
    goal.reference_frame_id = args.reference_frame

    action.send_goal(goal, done_cb=action_done, feedback_cb=feedback_cb)
    try:
        action.wait_for_result()
    except KeyboardInterrupt:
        rospy.loginfo("Cancelling goal")
        action.cancel_goal()
        rospy.sleep(1.0)


if __name__ == '__main__':
    main()
