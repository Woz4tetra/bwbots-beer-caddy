#!/usr/bin/env python3
import rospy
import actionlib
import argparse

from bw_interfaces.msg import RunBehaviorAction, RunBehaviorGoal, RunBehaviorFeedback, RunBehaviorResult
from bw_interfaces.msg import DrinkMission, DrinkMissionArray


def main():
    prev_feedback = RunBehaviorFeedback()
    def action_done(goal_status, result: RunBehaviorResult):
        rospy.loginfo(f"Behavior finished with result: {result.success}. Status: {goal_status}")

    def feedback_cb(feedback: RunBehaviorFeedback):
        nonlocal prev_feedback
        if feedback != prev_feedback:
            rospy.loginfo(f"Behavior feedback: {feedback}")
            prev_feedback = feedback
    
    def are_missions_equal(mission1: DrinkMission, mission2: DrinkMission) -> bool:
        return (
            mission1.delivery_waypoint_key == mission2.delivery_waypoint_key and
            mission1.drink_dispenser_tag == mission2.drink_dispenser_tag
        )
        
    def missions_callback(msg: DrinkMissionArray):
        nonlocal our_mission, mission_queued
        for mission in msg.missions:
            if are_missions_equal(mission, our_mission):
                mission_queued = True
    
    parser = argparse.ArgumentParser(description="action_script")

    parser.add_argument("drink_dispenser",
                        help="drink dispenser waypoint key")
    parser.add_argument("delivery_destination",
                        help="waypoint to deliver to")
    args = parser.parse_args()
    
    our_mission = DrinkMission(
        args.delivery_destination,
        args.drink_dispenser
    )
    mission_queued = False
    
    rospy.init_node(
        "delivery_mission_script",
        disable_signals=True,
        # log_level=rospy.DEBUG
    )
    missions_sub = rospy.Subscriber("/bw/missions", DrinkMissionArray, missions_callback, queue_size=5)
    
    add_mission_pub = rospy.Publisher("/bw/add_mission", DrinkMission, queue_size=5)
    rospy.sleep(0.1)
    add_mission_pub.publish(our_mission)
    
    start_time = rospy.Time.now()
    while rospy.Time.now() - start_time < rospy.Duration(3.0):
        if mission_queued:
            break
        rospy.sleep(0.1)
    if not mission_queued:
        raise RuntimeError("Failed to queue mission!")

    missions_sub.unregister()

    action = actionlib.SimpleActionClient("/bw/run_behavior", RunBehaviorAction)
    rospy.loginfo("Connecting to action server...")
    action.wait_for_server()

    goal = RunBehaviorGoal()
    goal.behavior = "drink_mission"

    action.send_goal(goal, done_cb=action_done, feedback_cb=feedback_cb)
    try:
        action.wait_for_result()
    except KeyboardInterrupt:
        rospy.loginfo("Cancelling goal")
        action.cancel_goal()
        rospy.sleep(1.0)


if __name__ == '__main__':
    main()
