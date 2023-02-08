#!/usr/bin/env python3
import rospy
import actionlib

from geometry_msgs.msg import Twist

from bw_interfaces.msg import GoToPoseAction, GoToPoseGoal, GoToPoseResult

from bw_tools.robot_state import Pose2d


def main():
    pose_result = GoToPoseResult()

    def pose_action_done(goal_status, result: GoToPoseResult):
        nonlocal pose_result
        pose_result = result
        rospy.loginfo(f"Pose action finished with result: {result.success}. Status: {goal_status}")

    def go_to_distance(distance, **kwargs):
        pose_goal = GoToPoseGoal()
        pose_goal.goal.pose = Pose2d(x=distance).to_ros_pose()
        pose_goal.goal.header.frame_id = "base_link"
        pose_goal.xy_tolerance = kwargs.get("xy_tolerance", 0.05)
        pose_goal.yaw_tolerance = kwargs.get("yaw_tolerance", 0.15)
        pose_goal.timeout = rospy.Duration(kwargs.get("timeout", 2.0))
        pose_goal.reference_linear_speed = kwargs.get("reference_linear_speed", 0.5)
        pose_goal.reference_angular_speed = kwargs.get("reference_angular_speed", 3.0)
        pose_goal.allow_reverse = kwargs.get("allow_reverse", True)
        pose_goal.rotate_in_place_start = kwargs.get("rotate_in_place_start", True)
        pose_goal.rotate_while_driving = kwargs.get("rotate_while_driving", True)
        pose_goal.rotate_in_place_end = kwargs.get("rotate_in_place_end", True)
        pose_goal.linear_max_accel = kwargs.get("linear_max_accel", 2.0)
        pose_goal.linear_min_vel = kwargs.get("linear_min_vel", 0.015)
        pose_goal.theta_max_accel = kwargs.get("theta_max_accel", 1.0)
        pose_goal.theta_min_vel = kwargs.get("theta_min_vel", 0.015)
        go_to_pose_action.send_goal(pose_goal, done_cb=pose_action_done)
        go_to_pose_action.wait_for_result()
        return pose_result.success

    def stop_driving():
        cmd_vel_pub.publish(Twist())

    rospy.init_node(
        "go_to_pose_action_script",
        disable_signals=True,
        # log_level=rospy.DEBUG
    )

    cmd_vel_pub = rospy.Publisher("/bw/cmd_vel", Twist, queue_size=10)

    go_to_pose_action = actionlib.SimpleActionClient("/bw/go_to_pose", GoToPoseAction)
    rospy.loginfo("Connecting to action servers...")
    go_to_pose_action.wait_for_server()
    rospy.loginfo("Action servers connected!")

    try:
        go_to_distance(
            -0.7,
            controller_type="strafe2",
            xy_tolerance=0.15,
            yaw_tolerance=0.15,
            linear_min_vel=0.4,
            reference_linear_speed=1.0,
            reference_angular_speed=3.0,
            linear_max_accel=5.0,
            allow_reverse=True,
            rotate_in_place_start=False,
            rotate_while_driving=False,
            rotate_in_place_end=False)

        stop_driving()

    except KeyboardInterrupt:
        rospy.loginfo("Cancelling goal")
        go_to_pose_action.cancel_goal()
        rospy.sleep(1.0)


if __name__ == '__main__':
    main()
