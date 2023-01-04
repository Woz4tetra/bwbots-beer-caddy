#!/usr/bin/env python3
import rospy
from commands.find_tag import FindTagCommand
from commands.go_to_pose import GoToPoseCommand
from commands.follow_waypoint import FollowWaypoint
from commands.shuffle_until_charging import ShuffleUntilChargingCommand
from commands.run_sequence import RunSequenceCommand
from commands.set_robot_state import SetRobotStateCommand
from commands.has_drink import HasDrinkCommand
from commands.follow_detection import FollowDetectionCommand
from simple_move_base_client import SimpleMoveBaseClient


def main():
    commands = {}
    node_name = "bw_behaviors"
    rospy.init_node(
        node_name,
        # disable_signals=True,
        log_level=rospy.DEBUG,
    )

    move_base_namespace = rospy.get_param("~move_base_namespace", "/move_base")
    global_frame = rospy.get_param("~global_frame", "map")
    base_frame = rospy.get_param("~base_frame", "base_link")
    move_base_client = SimpleMoveBaseClient(
        move_base_namespace, base_frame, global_frame
    )

    commands["follow_waypoint"] = FollowWaypoint(move_base_client)
    commands["find_tag"] = FindTagCommand()
    commands["go_to_pose"] = GoToPoseCommand()
    commands["shuffle_until_charging"] = ShuffleUntilChargingCommand()
    commands["run_sequence"] = RunSequenceCommand()
    commands["set_robot_state"] = SetRobotStateCommand()
    commands["has_drink"] = HasDrinkCommand()
    commands["follow_detection"] = FollowDetectionCommand(move_base_client)

    rospy.spin()


if __name__ == "__main__":
    main()
