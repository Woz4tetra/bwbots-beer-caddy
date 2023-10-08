#!/usr/bin/env python3
import argparse

import rospy

from bw_interfaces.srv import TeachWaypoint, TeachWaypointRequest, TeachWaypointResponse


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("name", help="Name of the waypoint to teach")
    args = parser.parse_args()

    rospy.init_node("teach_waypoint_script")

    teach_waypoint = rospy.ServiceProxy("/bw/teach_waypoint", TeachWaypoint)

    result: TeachWaypointResponse = teach_waypoint(TeachWaypointRequest(name=args.name))
    if result.success:
        print("Successfully taught waypoint %s" % args.name)
    else:
        print("Failed to teach waypoint %s" % args.name)


if __name__ == "__main__":
    main()
