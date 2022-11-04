#!/usr/bin/env python3
import rospy
import argparse

from bw_interfaces.srv import SaveTF


def main():
    rospy.init_node(
        "save_robot_waypoint_script",
        disable_signals=True,
        # log_level=rospy.DEBUG
    )

    save_tf_srv = rospy.ServiceProxy("/bw/bw_waypoints/save_tf", SaveTF)

    parser = argparse.ArgumentParser(description="action_script")

    parser.add_argument("name", type=str,
                        help="Name of waypoint to save")
    parser.add_argument("--map", default="map",
                        help="Name of map frame")
    parser.add_argument("--robot", default="base_link",
                        help="Name of robot base_link frame")
    args = parser.parse_args()

    success = save_tf_srv(args.name, args.map, args.robot)
    rospy.loginfo("Saving waypoint was %s" % ("successful" if success else "unsuccessful"))


if __name__ == '__main__':
    main()
