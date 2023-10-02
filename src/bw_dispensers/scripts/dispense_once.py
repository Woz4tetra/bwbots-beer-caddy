#!/usr/bin/env python3
import actionlib
import rospy

from bw_interfaces.msg import DispenseAction, DispenseFeedback, DispenseGoal, DispenseResult
from bw_tools.typing.basic import seconds_to_duration


def main():
    def action_done(goal_status, result: DispenseResult):
        rospy.loginfo(f"Action finished with result: {result.success}. Status: {goal_status}")

    def feedback_cb(feedback: DispenseFeedback):
        pass

    rospy.init_node("dispense_script", disable_signals=True)

    action = actionlib.SimpleActionClient("/bw/dispense", DispenseAction)
    rospy.loginfo("Connecting to action server...")
    action.wait_for_server()

    goal = DispenseGoal()

    goal.dispenser_name = "drum_dispenser"
    goal.timeout = seconds_to_duration(3.0)

    action.send_goal(goal, done_cb=action_done, feedback_cb=feedback_cb)
    try:
        action.wait_for_result()
    except KeyboardInterrupt:
        rospy.loginfo("Cancelling sequence")
        action.cancel_goal()
        rospy.sleep(1.0)


if __name__ == '__main__':
    main()
