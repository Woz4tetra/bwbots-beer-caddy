#!/usr/bin/env python3
import rospy
import actionlib
import argparse

from bw_interfaces.msg import SaveTagAsWaypointAction, SaveTagAsWaypointGoal, SaveTagAsWaypointFeedback, SaveTagAsWaypointResult


def main():
    prev_feedback = SaveTagAsWaypointFeedback()
    def action_done(goal_status, result: SaveTagAsWaypointResult):
        rospy.loginfo(f"Behavior finished with result: {result.success}. Status: {goal_status}")

    def feedback_cb(feedback: SaveTagAsWaypointFeedback):
        nonlocal prev_feedback
        if feedback != prev_feedback:
            rospy.loginfo(f"Behavior feedback: {feedback}")
            prev_feedback = feedback
        
    rospy.init_node(
        "save_tag_and_prep",
        disable_signals=True,
        # log_level=rospy.DEBUG
    )

    action = actionlib.SimpleActionClient("/bw/save_tag_as_waypoint", SaveTagAsWaypointAction)
    rospy.loginfo("Connecting to action server...")
    action.wait_for_server()

    parser = argparse.ArgumentParser(description="action_script")

    parser.add_argument("tag",
                        help="name of tag defined in tag_mapping.yaml")
    args = parser.parse_args()

    goal = SaveTagAsWaypointGoal()
    goal.waypoint_key = args.tag
    goal.prep_x_offset = -0.7
    goal.prep_y_offset = 0.0

    action.send_goal(goal, done_cb=action_done, feedback_cb=feedback_cb)
    try:
        action.wait_for_result()
    except KeyboardInterrupt:
        rospy.loginfo("Cancelling goal")
        action.cancel_goal()
        rospy.sleep(1.0)


if __name__ == '__main__':
    main()
