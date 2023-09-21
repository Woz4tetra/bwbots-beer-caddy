#!/usr/bin/env python3
import argparse
import os
from typing import List

import actionlib
import rospkg
import rospy
import tqdm
import yaml

from bw_interfaces.msg import RunSequenceAction, RunSequenceFeedback, RunSequenceGoal, RunSequenceResult


def load_sequences() -> List[str]:
    package_path = rospkg.RosPack().get_path("bw_lights_music")
    config_path = os.path.join(package_path, "config", "sequences.yaml")
    with open(config_path) as file:
        sequences = yaml.safe_load(file)
    return sequences


def main():
    def action_done(goal_status, result: RunSequenceResult):
        rospy.loginfo(f"Action finished with result: {result.success}. Status: {goal_status}")

    prev_index = 0
    length = 0
    pbar = None

    def feedback_cb(feedback: RunSequenceFeedback):
        nonlocal pbar, prev_index, length
        if pbar is None or length != feedback.length:
            length = feedback.length
            pbar = tqdm.tqdm(f"Sequence {args.sequence}", total=feedback.length, ascii=False)
            prev_index = 0
        delta = feedback.index - prev_index
        prev_index = feedback.index
        pbar.update(delta)

    rospy.init_node("run_sequence_script", disable_signals=True)
    sequences = load_sequences()

    parser = argparse.ArgumentParser(description="play_sequence")

    parser.add_argument("sequence", default="startup", choices=list(sequences), help="Sequence name")
    parser.add_argument("-l", "--loop", action="store_true", help="Whether to loop the sequence or not")
    args = parser.parse_args()

    action = actionlib.SimpleActionClient("/bw/run_sequence", RunSequenceAction)
    rospy.loginfo("Connecting to action server...")
    action.wait_for_server()

    goal = RunSequenceGoal()

    goal.name = args.sequence
    goal.loop = args.loop

    action.send_goal(goal, done_cb=action_done, feedback_cb=feedback_cb)
    try:
        action.wait_for_result()
    except KeyboardInterrupt:
        rospy.loginfo("Cancelling sequence")
        action.cancel_goal()
        rospy.sleep(1.0)


if __name__ == '__main__':
    main()
