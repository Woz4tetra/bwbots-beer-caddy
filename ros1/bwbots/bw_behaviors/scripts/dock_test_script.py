#!/usr/bin/env python3
import rospy
import actionlib

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist

from bw_interfaces.msg import FindTagAction, FindTagGoal, FindTagFeedback, FindTagResult
from bw_interfaces.msg import GoToPoseAction, GoToPoseGoal, GoToPoseResult
from bw_interfaces.msg import FollowWaypointsAction, FollowWaypointsGoal, FollowWaypointsResult
from bw_interfaces.srv import GetWaypoints

from bw_tools.robot_state import Pose2d


def main():
    tag_pose = Pose2d()
    pose_result = GoToPoseResult()
    waypoint_result = FollowWaypointsResult()

    def pose_action_done(goal_status, result: GoToPoseResult):
        nonlocal pose_result
        pose_result = result
        rospy.loginfo(f"Pose action finished with result: {result.success}. Status: {goal_status}")

    def tag_action_done(goal_status, result: FindTagResult):
        nonlocal tag_pose
        rospy.loginfo(f"Action finished with result: {result.success}. Status: {goal_status}")
        tag_pose = Pose2d.from_ros_pose(result.pose.pose)
        tag_result_pub.publish(result.pose)
        rospy.loginfo(f"Tag header: {result.pose.header.frame_id}")
        rospy.loginfo(f"Tag location: {{x: {tag_pose.x}, y: {tag_pose.y}, theta: {tag_pose.theta}}}")

    def waypoint_action_done(goal_status, result: FollowWaypointsResult):
        nonlocal waypoint_result
        rospy.loginfo(f"Waypoint action finished with result: {result.success}. Status: {goal_status}")
        waypoint_result = result

    def feedback_cb(feedback: FindTagFeedback):
        tag_sample_pub.publish(feedback.sample)
    
    def go_to_tag(distance_offset, **kwargs):
        offset = Pose2d(0.0, distance_offset, 1.5708)
        pose2d = offset.transform_by(tag_pose)

        pose_goal = GoToPoseGoal()
        pose_goal.goal.pose = pose2d.to_ros_pose()
        pose_goal.goal.header.frame_id = tag_goal.reference_frame_id
        pose_goal.xy_tolerance = kwargs.get("xy_tolerance", 0.05)
        pose_goal.yaw_tolerance = kwargs.get("yaw_tolerance", 0.15)
        pose_goal.timeout = rospy.Duration(kwargs.get("timeout", 2.0))
        pose_goal.reference_linear_speed = kwargs.get("reference_linear_speed", 0.5)
        pose_goal.reference_angular_speed = kwargs.get("reference_angular_speed", 3.0)
        pose_goal.allow_reverse = kwargs.get("allow_reverse", True)
        pose_goal.rotate_in_place_start = kwargs.get("rotate_in_place_start", True)
        pose_goal.rotate_in_place_end = kwargs.get("rotate_in_place_end", True)
        go_to_pose_action.send_goal(pose_goal, done_cb=pose_action_done)
        go_to_pose_action.wait_for_result()
        return pose_result.success

    def drive_straight(vx, duration: float):
        twist = Twist()
        twist.linear.x = vx
        start = rospy.Time.now()
        while rospy.Time.now() - start < rospy.Duration(duration):
            cmd_vel_pub.publish(twist)
            rospy.sleep(0.005)
    
    def stop_driving():
        cmd_vel_pub.publish(Twist())

    def get_waypoint(name):
        rospy.loginfo(f"Requesting locations of waypoint: {name}")
        result = get_waypoints_srv([name])
        if not result.success:
            rospy.logwarn("Failed to get waypoint locations")
            return None
        rospy.loginfo(f"Waypoint locations: {result}")
        waypoints_array = result.waypoints
        return waypoints_array

    def go_to_waypoint(name):
        goal = FollowWaypointsGoal()
        waypoints_array = get_waypoint(name)
        if waypoints_array is None:
            return False
        goal.waypoints = waypoints_array

        waypoint_action.send_goal(goal, done_cb=waypoint_action_done)
        waypoint_action.wait_for_result()
        return waypoint_result.success

    rospy.init_node(
        "go_to_pose_action_script",
        disable_signals=True,
        # log_level=rospy.DEBUG
    )

    tag_sample_pub = rospy.Publisher("/bw/tag_sample", PoseStamped, queue_size=10)
    tag_result_pub = rospy.Publisher("/bw/tag_pose", PoseStamped, queue_size=10)
    cmd_vel_pub = rospy.Publisher("/bw/cmd_vel", Twist, queue_size=10)
    
    get_waypoints_srv = rospy.ServiceProxy("/bw/bw_waypoints/get_waypoints", GetWaypoints)

    waypoint_action = actionlib.SimpleActionClient("/bw/follow_waypoints", FollowWaypointsAction)
    find_tag_action = actionlib.SimpleActionClient("/bw/find_tag", FindTagAction)
    go_to_pose_action = actionlib.SimpleActionClient("/bw/go_to_pose", GoToPoseAction)
    rospy.loginfo("Connecting to action server...")
    find_tag_action.wait_for_server()
    go_to_pose_action.wait_for_server()
    waypoint_action.wait_for_server()

    tag_goal = FindTagGoal()
    tag_goal.tag_id = []
    tag_goal.reference_frame_id = "map"
    
    try:
        if not go_to_waypoint("dock_prep"):
            return

        rospy.sleep(1.5)

        find_tag_action.send_goal(tag_goal, done_cb=tag_action_done, feedback_cb=feedback_cb)
        find_tag_action.wait_for_result()

        if not go_to_tag(-0.5, xy_tolerance=0.1, yaw_tolerance=0.01, timeout=10.0, reference_linear_speed=0.35, reference_angular_speed=3.0, allow_reverse=True):
            return
    
        drive_straight(0.02, 0.25)
        stop_driving()

        rospy.sleep(1.5)

        find_tag_action.send_goal(tag_goal, done_cb=tag_action_done, feedback_cb=feedback_cb)
        find_tag_action.wait_for_result()

        if not go_to_tag(-0.2, xy_tolerance=0.05, yaw_tolerance=0.05, reference_linear_speed=1.0, reference_angular_speed=3.0, allow_reverse=False, rotate_in_place_start=False, rotate_in_place_end=False):
            return

        drive_straight(0.75, 0.35)
        stop_driving()

    except KeyboardInterrupt:
        rospy.loginfo("Cancelling goal")
        find_tag_action.cancel_goal()
        go_to_pose_action.cancel_goal()
        rospy.sleep(1.0)


if __name__ == '__main__':
    main()