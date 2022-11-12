#!/usr/bin/env python3
import rospy
import argparse
import actionlib

from bw_interfaces.msg import RunSequenceAction, RunSequenceGoal, RunSequenceFeedback, RunSequenceResult


def main():
    def action_done(goal_status, result: RunSequenceResult):
        rospy.loginfo(f"Action finished with result: {result.success}. Status: {goal_status}")
        # tag_result_pub.publish(result.pose)

    def feedback_cb(feedback: RunSequenceFeedback):
        rospy.loginfo(f"Sequence: {feedback}")

    rospy.init_node(
        "run_sequence_script",
        disable_signals=True,
        # log_level=rospy.DEBUG
    )
    
    sequences = dict(
        startup=0,
        charging=1,
        driving=2,
        hotspot=3,
        hotspot_charging=4,
        idle=5,
        bot99=6,
        fur_elise=7,
        brawl=8,
        mii_channel=9,
        megalovania=10,
        ruins=11,
        scale1=12,
        scale2=13,
        scale3=14,
        nokia=15
    )

    action = actionlib.SimpleActionClient("/bw/run_sequence", RunSequenceAction)
    rospy.loginfo("Connecting to action server...")
    action.wait_for_server()

    parser = argparse.ArgumentParser(description="action_script")

    parser.add_argument("sequence",
                        default="startup",
                        help="Sequence name")
    parser.add_argument("-l", "--loop",
                        action="store_true",
                        help="Whether to loop the sequence or not")
    args = parser.parse_args()

    goal = RunSequenceGoal()
    try:
        serial = int(args.sequence)
        from_flash = False
    except ValueError:
        serial = sequences[args.sequence]
        from_flash = True
    
    goal.serial = serial
    goal.loop = args.loop
    goal.from_flash = from_flash

    action.send_goal(goal, done_cb=action_done, feedback_cb=feedback_cb)
    try:
        action.wait_for_result()
    except KeyboardInterrupt:
        rospy.loginfo("Cancelling sequence")
        action.cancel_goal()
        rospy.sleep(1.0)


if __name__ == '__main__':
    main()
