#!/usr/bin/env python3
from abc import ABC, abstractmethod
from typing import Optional, Tuple, cast

import actionlib
import rospy
import tf2_ros
from geometry_msgs.msg import PoseStamped, Twist

from bw_interfaces.msg import GoToPoseAction, GoToPoseFeedback
from bw_interfaces.msg import GoToPoseGoal as RosGoToPoseGoal
from bw_interfaces.msg import GoToPoseResult
from bw_tools.robot_state import Pose2d, Pose2dStamped, Velocity
from bw_tools.structs.go_to_goal import GoToPoseGoal
from bw_tools.transforms import lookup_pose, lookup_pose_in_frame
from bw_tools.typing.basic import get_param


class NavigationAction(ABC):
    def __init__(self) -> None:
        self.cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
        self.goal_pose_pub = rospy.Publisher("go_to_pose_goal", PoseStamped, queue_size=10)
        self.goal_pose_sub = None

        self.loop_rate = get_param("~loop_rate", 50.0)
        self.loop_period = 1.0 / self.loop_rate

        self.settling_time = get_param("~settling_time", 3.0)
        self.rotate_angle_threshold = get_param("~rotate_angle_threshold", 0.785)  # ~45 degrees
        self.strafe_angle_threshold = get_param("~strafe_angle_threshold", 0.5)  # ~30 degrees

        self.global_frame = get_param("~global_frame", "map")
        self.robot_frame = get_param("~robot_frame", "base_link")

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.action_server = actionlib.SimpleActionServer(
            "go_to_pose",
            GoToPoseAction,
            execute_cb=self.action_callback,
            auto_start=False,
        )
        self.action_server.start()
        rospy.loginfo("go_to_pose is ready")

    def get_robot_state(self) -> Optional[Pose2dStamped]:
        pose_stamped = lookup_pose(self.tf_buffer, self.robot_frame, self.global_frame, silent=True)
        if pose_stamped is None:
            return None
        else:
            return Pose2dStamped.from_msg(pose_stamped)

    def compute_goal(self, msg: Pose2dStamped) -> Optional[Pose2dStamped]:
        robot_state = self.get_robot_state()
        if robot_state is None:
            rospy.logwarn("No robot pose received. Can't set goal.")
            return None
        goal = lookup_pose_in_frame(self.tf_buffer, msg.to_msg(), robot_state.header.frame_id)
        if goal is None:
            rospy.logwarn("Failed to look up goal in global frame. Can't set goal.")
            return None
        return Pose2dStamped.from_msg(goal)

    def publish_velocity(self, velocity: Velocity):
        twist = Twist()
        twist.linear.x = velocity.x
        twist.linear.y = velocity.y
        twist.angular.z = velocity.theta
        self.cmd_vel_pub.publish(twist)

    def publish_state_feedback(self, current_pose2d: Pose2dStamped, goal_pose2d: Pose2dStamped):
        current_pose = current_pose2d.to_msg()
        goal_pose = goal_pose2d.to_msg()

        feedback = GoToPoseFeedback()
        feedback.current_pose.header = current_pose.header
        feedback.current_pose.pose = current_pose.pose
        feedback.goal_pose.header = goal_pose.header
        feedback.goal_pose.pose = goal_pose.pose
        self.action_server.publish_feedback(feedback)
        self.goal_pose_pub.publish(goal_pose)

    def get_error(self, goal_pose: Pose2d, robot_state: Pose2d) -> Pose2d:
        return cast(Pose2d, goal_pose.relative_to(robot_state))

    @abstractmethod
    def controller_init(self, goal: GoToPoseGoal):
        pass

    @abstractmethod
    def controller_update(self, goal: GoToPoseGoal, robot_state: Pose2dStamped) -> Tuple[Velocity, bool]:
        pass

    def action_callback(self, msg: RosGoToPoseGoal):
        goal = GoToPoseGoal.from_msg(msg)
        rospy.loginfo(f"Going to pose: {goal}")

        robot_state = self.get_robot_state()
        xy_tolerance: float = goal.xy_tolerance
        yaw_tolerance: float = goal.yaw_tolerance
        timeout: rospy.Duration = goal.timeout  # type: ignore

        computed_goal_pose = self.compute_goal(goal.goal)
        if computed_goal_pose is None:
            rospy.loginfo("Failed to compute goal pose")
            self.action_server.set_succeeded(GoToPoseResult(False), "Failed to compute goal pose")
            return

        robot_state = self.get_robot_state()
        rospy.logdebug(f"Set goal pose to {goal.goal}")
        rospy.logdebug(f"Robot is at {robot_state}")

        rate = rospy.Rate(self.loop_rate)
        start_time = rospy.Time.now()
        current_time = rospy.Time.now()
        success = False
        aborted = False
        while current_time - start_time < timeout:
            rate.sleep()
            current_time = rospy.Time.now()

            if self.action_server.is_preempt_requested():
                rospy.loginfo("Cancelling go to pose")
                aborted = True
                break

            robot_state = self.get_robot_state()
            if robot_state is None:
                continue

            velocity_command, is_done = self.controller_update(goal, robot_state)

            heading = self.get_error(goal.goal.pose, robot_state.pose).heading()
            rospy.logdebug(
                f"Error: {self.get_error(goal.goal.pose, robot_state.pose)}. "
                f"Heading: {heading} Velocity: {velocity_command}. "
                f"Goal: {goal.goal}. "
                f"State: {robot_state}"
            )
            self.publish_velocity(velocity_command)
            self.publish_state_feedback(robot_state, goal.goal)

            if is_done:
                rospy.loginfo(f"Controller finished. Pose error: {goal.goal.pose - robot_state.pose}")
                success = True
                break

        self.publish_velocity(Velocity())
        if current_time - start_time > timeout:
            rospy.loginfo("Go to pose timed out")

        if robot_state is None:
            distance = 0.0
            angle_error = 0.0
            rospy.loginfo("Never received robot's position")
        else:
            error = self.get_error(goal.goal.pose, robot_state.pose)
            distance = error.magnitude()
            angle_error = abs(error.theta)

        if distance > xy_tolerance or angle_error > yaw_tolerance:
            success = False

        result = GoToPoseResult(success)
        if distance > xy_tolerance:
            rospy.loginfo(f"Distance tolerance not met: {distance} > {xy_tolerance}")
        else:
            rospy.loginfo(f"Distance tolerance met: {distance} <= {xy_tolerance}")
        if angle_error > yaw_tolerance:
            rospy.loginfo(f"Angle tolerance not met: {angle_error} > {yaw_tolerance}")
        else:
            rospy.loginfo(f"Angle tolerance met: {angle_error} <= {yaw_tolerance}")

        if aborted:
            self.action_server.set_aborted(result, "Interrupted while going to a pose")
        else:
            if result.success:
                rospy.loginfo("Return success for go to pose")
            else:
                rospy.loginfo("Return failure for go to pose")
            self.action_server.set_succeeded(result)

    def run(self) -> None:
        rospy.spin()
