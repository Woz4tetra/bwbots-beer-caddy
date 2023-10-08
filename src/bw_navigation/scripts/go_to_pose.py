import argparse
from threading import Event

import actionlib
import rospy
import yaml
from py_trees.common import Status

from bw_interfaces.msg import GoToPoseAction, GoToPoseFeedback, GoToPoseGoal, GoToPoseResult
from bw_tools.robot_state import Pose2d


class GoToPoseManager:
    def __init__(self):
        self.action = actionlib.SimpleActionClient("/bw/go_to_pose", GoToPoseAction)
        self.action.wait_for_server()
        self.status = Status.FAILURE
        self.event = Event()
        rospy.loginfo("Simple go to pose action connected")

    def send_goal(self, goal: GoToPoseGoal) -> None:
        self.action.send_goal(goal, done_cb=self.action_done, feedback_cb=self.feedback_cb)
        self.event.clear()

    def wait(self) -> None:
        self.event.wait()

    def action_done(self, goal_status, result: GoToPoseResult) -> None:
        self.event.set()
        rospy.loginfo(f"Action finished with result: {result.success}. Status: {goal_status}")

    def feedback_cb(self, feedback: GoToPoseFeedback) -> None:
        pass

    def cancel(self) -> None:
        self.action.cancel_all_goals()
        self.event.clear()


def main():
    parser = argparse.ArgumentParser(description="action_script")

    parser.add_argument(
        "pose",
        type=str,
        help="Goal pose. Yaml format (ex. \"{x: 0.1}\". Equivalent to \"{x: 0.1, y: 0.0, theta: 0.0}\")",
    )
    parser.add_argument("-xy", "--xy-tolerance", default=0.01, type=float, help="Distance tolerance")
    parser.add_argument("-th", "--theta-tolerance", default=0.05, type=float, help="Angular tolerance")
    parser.add_argument("-t", "--timeout", default=5.0, type=float, help="Action timeout")
    parser.add_argument("-io", "--ignore-obstacles", default=False, type=bool, help="Ignore obstacles during movement")
    parser.add_argument("-l", "--reference-linear-speed", default=0.5, type=float, help="Linear movement speed")
    parser.add_argument("-a", "--reference-angular-speed", default=3.0, type=float, help="Angular movement speed")
    parser.add_argument("-f", "--reference-frame", default="base_link", type=str, help="Parent frame of pose")
    parser.add_argument("-r", "--allow_reverse", action="store_true", help="Allow backwards motion")
    args = parser.parse_args()

    rospy.init_node("go_to_pose_action_script")

    go_to_pose_manager = GoToPoseManager()

    pose_dict: dict = yaml.safe_load(args.pose)
    pose2d = Pose2d.from_xyt(**pose_dict)

    goal = GoToPoseGoal()
    goal.xy_tolerance = args.xy_tolerance
    goal.yaw_tolerance = args.theta_tolerance
    goal.timeout = rospy.Duration(args.timeout)
    goal.reference_linear_speed = args.reference_linear_speed
    goal.reference_angular_speed = args.reference_angular_speed
    goal.allow_reverse = args.allow_reverse
    goal.rotate_in_place_start = True
    goal.rotate_while_driving = True
    goal.rotate_in_place_end = True
    goal.linear_max_accel = 0.25
    goal.linear_min_vel = 0.1
    goal.theta_max_accel = 1.0
    goal.theta_min_vel = 0.15
    goal.goal.pose = pose2d.to_ros_pose()
    goal.goal.header.frame_id = args.reference_frame

    go_to_pose_manager.send_goal(goal)

    try:
        rospy.loginfo("Waiting for go to pose routine to finish")
        go_to_pose_manager.wait()
        rospy.loginfo("done!")
    except KeyboardInterrupt:
        rospy.loginfo("Cancelling goal")
        go_to_pose_manager.cancel()
        rospy.sleep(1.0)


if __name__ == '__main__':
    main()
