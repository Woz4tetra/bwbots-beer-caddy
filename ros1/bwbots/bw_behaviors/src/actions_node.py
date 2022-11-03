import rospy

from bw_actions.go_to_waypoint import GoToWaypointAction


def main():
    actions = {}
    node_name = "bw_waypoints"
    rospy.init_node(
        node_name
        # disable_signals=True
        # log_level=rospy.DEBUG
    )
    
    actions["go_to_waypoint"] = GoToWaypointAction()

    rospy.spin()


if __name__ == "__main__":
    main()
