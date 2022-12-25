#!/usr/bin/env python3
import rospy
from commands.find_tag import FindTagCommand
from commands.go_to_pose import GoToPoseCommand
from commands.follow_waypoint import FollowWaypoint
from commands.shuffle_until_charging import ShuffleUntilChargingCommand
from commands.run_sequence import RunSequenceCommand
from commands.set_robot_state import SetRobotStateCommand
from commands.has_drink import HasDrinkCommand


def main():
    commands = {}
    node_name = "bw_behaviors"
    rospy.init_node(
        node_name,
        # disable_signals=True,
        log_level=rospy.DEBUG
    )
    
    commands["follow_waypoint"] = FollowWaypoint()
    commands["find_tag"] = FindTagCommand()
    commands["go_to_pose"] = GoToPoseCommand()
    commands["shuffle_until_charging"] = ShuffleUntilChargingCommand()
    commands["run_sequence"] = RunSequenceCommand()
    commands["set_robot_state"] = SetRobotStateCommand()
    commands["has_drink"] = HasDrinkCommand()

    rospy.spin()


if __name__ == "__main__":
    main()
