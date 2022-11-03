import rospy

from commands.go_to_waypoint import GoToWaypointCommand
from commands.find_tag import FindTagCommand


def main():
    actions = {}
    node_name = "bw_behaviors"
    rospy.init_node(
        node_name
        # disable_signals=True
        # log_level=rospy.DEBUG
    )
    
    actions["go_to_waypoint"] = GoToWaypointCommand()
    actions["find_tag"] = FindTagCommand()

    rospy.spin()


if __name__ == "__main__":
    main()
