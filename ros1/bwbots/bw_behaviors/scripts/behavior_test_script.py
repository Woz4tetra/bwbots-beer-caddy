#!/usr/bin/env python3
import rospy
import actionlib
import argparse

from bw_interfaces.msg import RunBehaviorAction, RunBehaviorGoal, RunBehaviorFeedback, RunBehaviorResult


def main():
    prev_feedback = RunBehaviorFeedback()
    def action_done(goal_status, result: RunBehaviorResult):
        rospy.loginfo(f"Behavior finished with result: {result.success}. Status: {goal_status}")

    def feedback_cb(feedback: RunBehaviorFeedback):
        nonlocal prev_feedback
        if feedback != prev_feedback:
            rospy.loginfo(f"Behavior feedback: {feedback}")
            prev_feedback = feedback
        
    rospy.init_node(
        "behavior_action_script",
        disable_signals=True,
        # log_level=rospy.DEBUG
    )

    action = actionlib.SimpleActionClient("/bw/run_behavior", RunBehaviorAction)
    rospy.loginfo("Connecting to action server...")
    action.wait_for_server()

    parser = argparse.ArgumentParser(description="action_script")

    parser.add_argument("behavior",
                        help="Behavior to run")
    args = parser.parse_args()

    goal = RunBehaviorGoal()
    goal.behavior = args.behavior

    action.send_goal(goal, done_cb=action_done, feedback_cb=feedback_cb)
    try:
        action.wait_for_result()
    except KeyboardInterrupt:
        rospy.loginfo("Cancelling goal")
        action.cancel_goal()
        rospy.sleep(1.0)


if __name__ == '__main__':
    main()
