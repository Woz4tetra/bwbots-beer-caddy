import yaml
import rospy
import actionlib
import argparse

from bw_interfaces.msg import FollowWaypointsAction, FollowWaypointsGoal, FollowWaypointsResult
from bw_interfaces.srv import GetWaypoints


def action_done(result: FollowWaypointsResult):
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

    get_waypoints_srv = rospy.ServiceProxy("/bw/get_waypoints", GetWaypoints)

    parser = argparse.ArgumentParser(description="action_script")

    parser.add_argument("waypoints", nargs='+', type=str, default=[],
                        help="Names of waypoints to follow")
    args = parser.parse_args()

    rospy.loginfo(f"Requesting locations of waypoints: {args.waypoints}")
    waypoints_array = get_waypoints_srv(args.waypoints)
    rospy.loginfo(f"Waypoint locations: {waypoints_array}")

    goal = FollowWaypointsGoal()
    goal.waypoints = waypoints_array

    action.send_goal(goal, done_cb=action_done)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Cancelling goal")
        action.cancel_goal()
