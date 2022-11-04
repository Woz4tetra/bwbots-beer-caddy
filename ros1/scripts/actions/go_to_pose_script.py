import yaml
import rospy
import actionlib
import argparse

from bw_interfaces.msg import GoToPoseAction, GoToPoseGoal, GoToPoseResult

from bw_tools.robot_state import Pose2d

def action_done(result: GoToPoseResult):
    rospy.loginfo(f"Action finished with result: {result}")


def main():
    rospy.init_node(
        "go_to_pose_action_script",
        disable_signals=True,
        # log_level=rospy.DEBUG
    )
    
    action = actionlib.SimpleActionClient("/bw/go_to_pose", GoToPoseAction)
    rospy.loginfo("Connecting to action server...")
    action.wait_for_server()

    parser = argparse.ArgumentParser(description="action_script")

    parser.add_argument("pose",
                        help="Goal pose. Yaml format (ex. \"{x: 0.1}\". Equivalent to \"{x: 0.1, y: 0.0, theta: 0.0}\")")
    parser.add_argument("-xy", "--xy-tolerance",
                        default=0.25,
                        help="Distance tolerance")
    parser.add_argument("-th", "--theta-tolerance",
                        default=0.25,
                        help="Angular tolerance")
    parser.add_argument("-t", "--timeout",
                        default=10.0,
                        help="Action timeout")
    parser.add_argument("io", "--ignore-obstacles",
                        default=False,
                        help="Ignore obstacles during movement")
    parser.add_argument("-l", "--reference-linear-speed",
                        default=0.5,
                        help="Linear movement speed")
    parser.add_argument("-a", "--reference-angular-speed",
                        default=3.0,
                        help="Angular movement speed")
    parser.add_argument("-f", "--reference-frame",
                        default="odom",
                        help="Parent frame of pose")
    args = parser.parse_args()

    pose_dict: dict = yaml.safe_load(args.pose)
    pose2d = Pose2d.from_xyt(**pose_dict)

    goal = GoToPoseGoal()
    goal.goal.pose = pose2d.to_ros_pose()
    goal.goal.header.frame_id = args.reference_frame
    goal.xy_tolerance = args.xy_tolerance
    goal.yaw_tolerance = args.theta_tolerance
    goal.timeout = rospy.Duration(args.timeout)
    goal.ignore_obstacles = args.ignore_obstacles
    goal.reference_linear_speed = args.reference_linear_speed
    goal.reference_angular_speed = args.reference_angular_speed

    action.send_goal(goal, done_cb=action_done)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Cancelling goal")
        action.cancel_goal()
