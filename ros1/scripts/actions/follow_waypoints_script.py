#!/usr/bin/env python3
import yaml
import rospy
import actionlib
import argparse

from bw_interfaces.msg import FollowWaypointsAction, FollowWaypointsGoal, FollowWaypointsResult
from bw_interfaces.srv import GetWaypoints


IS_DONE = False

def action_done(goal_status, result: FollowWaypointsResult):
    global IS_DONE
    IS_DONE = True
    rospy.loginfo(f"Action finished with result: {result}")


def main():
    rospy.init_node(
        "follow_waypoints_action_script",
        disable_signals=True,
        # log_level=rospy.DEBUG
    )
    
    action = actionlib.SimpleActionClient("/bw/follow_waypoints", FollowWaypointsAction)
    rospy.loginfo("Connecting to action server...")
    action.wait_for_server()

    get_waypoints_srv = rospy.ServiceProxy("/bw/bw_waypoints/get_waypoints", GetWaypoints)

    parser = argparse.ArgumentParser(description="action_script")

    parser.add_argument("waypoints", nargs='+', type=str, default=[],
                        help="Names of waypoints to follow")
    args = parser.parse_args()

    rospy.loginfo(f"Requesting locations of waypoints: {args.waypoints}")
    result = get_waypoints_srv(args.waypoints)
    if not result.success:
        rospy.logwarn("Failed to get waypoint locations")
        return
    rospy.loginfo(f"Waypoint locations: {result}")
    waypoints_array = result.waypoints

    goal = FollowWaypointsGoal()
    goal.waypoints = waypoints_array

    action.send_goal(goal, done_cb=action_done)
    try:
        while not IS_DONE:
            rospy.sleep(0.1)
    except KeyboardInterrupt:
        rospy.loginfo("Cancelling goal")
        action.cancel_goal()


if __name__ == '__main__':
    main()
