from typing import Optional
from bw_tools.controller.bwbots_controller import BwbotsController

import rospy
import actionlib

from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry

from bw_interfaces.msg import (
    GoToPoseAction,
    GoToPoseGoal,
    GoToPoseFeedback,
    GoToPoseResult,
)

from bw_tools.robot_state import Pose2d, Velocity
from bw_tools.controller.data import TrapezoidalProfileConfig, ControllerStateMachineConfig


class GoToPoseCommand:
    def __init__(self) -> None:
        self.robot_state: Optional[Pose2d] = None

        self.cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
        self.goal_pose_pub = rospy.Publisher(
            "go_to_pose_goal", PoseStamped, queue_size=10
        )
        self.goal_pose_sub = None

        self.loop_rate = rospy.get_param("~go_to_pose/loop_rate", 50.0)
        self.loop_period = 1.0 / self.loop_rate

        self.settling_time = rospy.get_param("~go_to_pose/settling_time", 3.0)
        self.rotate_angle_threshold = rospy.get_param("~go_to_pose/rotate_angle_threshold", 0.785)  # ~45 degrees

        self.goal_pose: Optional[Pose2d] = None
        self.robot_state: Optional[Pose2d] = None
        self.robot_parent_frame_id = ""
        self.robot_child_frame_id = ""

        self.odom_sub = rospy.Subscriber(
            "odom", Odometry, self.odom_callback, queue_size=10
        )

        self.goal_pose_sub = rospy.Subscriber(
            "moving_goal", PoseStamped, self.moving_goal_callback, queue_size=10
        )

        self.action_server = actionlib.SimpleActionServer(
            "go_to_pose",
            GoToPoseAction,
            execute_cb=self.action_callback,
            auto_start=False,
        )
        self.action_server.start()
        rospy.loginfo("go_to_pose is ready")

    def odom_callback(self, msg: Odometry) -> None:
        self.robot_state = Pose2d.from_ros_pose(msg.pose.pose)
        self.robot_parent_frame_id = msg.header.frame_id
        self.robot_child_frame_id = msg.child_frame_id

    def moving_goal_callback(self, msg: PoseStamped) -> None:
        self.goal_pose = self.compute_goal(msg)
        rospy.loginfo(f"Set goal pose to {self.goal_pose}")
    
    def compute_goal(self, msg: PoseStamped) -> Optional[Pose2d]:
        if self.robot_state is None:
            rospy.logwarn("No odometry received. Can't set goal.")
            return None
        if msg.header.frame_id == self.robot_parent_frame_id:
            return Pose2d.from_ros_pose(msg.pose)
        elif msg.header.frame_id == self.robot_child_frame_id:
            pose2d = Pose2d.from_ros_pose(msg.pose)
            return pose2d.transform_by(self.robot_state)
        else:
            rospy.logwarn(
                "Moving goal frame id doesn't match odometry. Can't set goal."
                f"Received {msg.header.frame_id}."
            )
            return None

    def publish_velocity(self, velocity: Velocity):
        twist = Twist()
        twist.linear.x = velocity.x
        twist.linear.y = velocity.y
        twist.angular.z = velocity.theta
        self.cmd_vel_pub.publish(twist)

    def publish_state_feedback(
        self,
        current_pose2d: Pose2d,
        goal_pose2d: Pose2d,
        current_header: str,
        goal_header: str,
    ):
        feedback = GoToPoseFeedback()
        feedback.current_pose.header.frame_id = current_header
        feedback.current_pose.pose = current_pose2d.to_ros_pose()
        feedback.goal_pose.header.frame_id = goal_header
        feedback.goal_pose.pose = goal_pose2d.to_ros_pose()
        self.action_server.publish_feedback(feedback)

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = self.robot_parent_frame_id
        goal_pose.pose = goal_pose2d.to_ros_pose()
        self.goal_pose_pub.publish(goal_pose)

    def action_callback(self, goal: GoToPoseGoal):
        rospy.loginfo(f"Going to pose: {goal}")

        xy_tolerance: float = goal.xy_tolerance
        yaw_tolerance: float = goal.yaw_tolerance
        timeout: rospy.Duration = goal.timeout
        reference_linear_speed: float = goal.reference_linear_speed
        reference_angular_speed: float = goal.reference_angular_speed
        allow_reverse = goal.allow_reverse
        rotate_in_place_start = goal.rotate_in_place_start
        rotate_while_driving = goal.rotate_while_driving
        rotate_in_place_end = goal.rotate_in_place_end
        linear_max_accel = goal.linear_max_accel
        theta_max_accel = goal.theta_max_accel
        linear_min_vel = goal.linear_min_vel
        theta_min_vel = goal.theta_min_vel
        
        linear_trapezoid = TrapezoidalProfileConfig(
            linear_min_vel,
            reference_linear_speed,
            linear_max_accel
        )
        angular_trapezoid = TrapezoidalProfileConfig(
            theta_min_vel,
            reference_angular_speed,
            theta_max_accel
        )
        pose_tolerance = Pose2d(xy_tolerance, xy_tolerance, yaw_tolerance)
        controller_config = ControllerStateMachineConfig(
            allow_reverse,
            rotate_while_driving,
            angular_trapezoid,
            linear_trapezoid,
            pose_tolerance,
            settle_time=self.settling_time,
            rotate_in_place_start=rotate_in_place_start,
            rotate_in_place_end=rotate_in_place_end,
            rotate_angle_threshold=self.rotate_angle_threshold
        )

        controller = BwbotsController(controller_config)

        rate = rospy.Rate(self.loop_rate)
        start_time = rospy.Time.now()
        current_time = rospy.Time.now()
        success = False
        aborted = False
        while current_time - start_time < timeout:
            rate.sleep()
            current_time = rospy.Time.now()

            if self.robot_state is None:
                continue
            self.goal_pose = self.compute_goal(goal.goal)
            if self.goal_pose is None:
                continue

            velocity_command, is_done = controller.compute(self.goal_pose, self.robot_state)

            rospy.logdebug(f"Robot pose: {self.robot_state}")
            rospy.logdebug(f"Goal pose: {self.goal_pose}")
            rospy.logdebug(f"Velocity command: {velocity_command}")
            self.publish_velocity(velocity_command)
            self.publish_state_feedback(
                self.robot_state,
                self.goal_pose,
                self.robot_parent_frame_id,
                self.robot_child_frame_id,
            )
            if self.action_server.is_preempt_requested():
                rospy.loginfo(f"Cancelling go to pose")
                aborted = True
                break
            if is_done:
                rospy.loginfo(
                    f"Controller finished. Pose error: {self.goal_pose - self.robot_state}"
                )
                success = True
                break

        if current_time - start_time > timeout:
            rospy.loginfo("Go to pose timed out")

        if self.robot_state is None or self.goal_pose is None:
            distance = 0.0
            angle_error = 0.0
            rospy.loginfo("Never received robot's position or goal")
        else:
            distance = self.robot_state.distance(self.goal_pose)
            angle_error = abs(self.robot_state.theta - self.goal_pose.theta)

        if distance > xy_tolerance or angle_error > yaw_tolerance:
            success = False
            
        result = GoToPoseResult(success)
        rospy.loginfo(
            f"Distance tolerance{' not' if distance > xy_tolerance else ''} met: {distance} > {xy_tolerance}"
        )
        rospy.loginfo(
            f"Angle tolerance{' not' if angle_error > yaw_tolerance else ''} met: {angle_error} > {yaw_tolerance}"
        )

        if aborted:
            self.action_server.set_aborted(result, "Interrupted while going to a pose")
        else:
            if result.success:
                rospy.loginfo("Return success for go to pose")
            else:
                rospy.loginfo("Return failure for go to pose")
            self.action_server.set_succeeded(result)
        self.robot_state = None
