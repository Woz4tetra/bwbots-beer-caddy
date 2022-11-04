#!/usr/bin/env python3
import rospy

from commands.go_to_waypoint import GoToWaypointCommand
from commands.find_tag import FindTagCommand
from commands.go_to_pose import GoToPoseCommand
from commands.shuffle_until_charging import ShuffleUntilChargingCommand


def main():
    commands = {}
    node_name = "bw_behaviors"
    rospy.init_node(
        node_name
        # disable_signals=True
        # log_level=rospy.DEBUG
    )
    
    commands["go_to_waypoint"] = GoToWaypointCommand()
    commands["find_tag"] = FindTagCommand()
    commands["go_to_pose"] = GoToPoseCommand()
    commands["shuffle_until_charging"] = ShuffleUntilChargingCommand()

    rospy.spin()


if __name__ == "__main__":
    main()
