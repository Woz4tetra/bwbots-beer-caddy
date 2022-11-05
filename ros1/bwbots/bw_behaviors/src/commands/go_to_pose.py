from typing import Optional
from enum import Enum

import math
import rospy
import tf2_ros
import actionlib

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist

import tf2_geometry_msgs

from bw_interfaces.msg import GoToPoseAction, GoToPoseGoal, GoToPoseFeedback, GoToPoseResult

from bw_tools.robot_state import Pose2d, Velocity
from bw_tools.controller.holonomic_drive_controller import HolonomicDriveController
from bw_tools.controller.nonholonomic_drive_controller import NonHolonomicDriveController
from bw_tools.controller.strafe_controller import StrafeController
from bw_tools.controller.ramsete_controller import RamseteController
from bw_tools.controller.profiled_pid import ProfiledPIDController
from bw_tools.controller.pid import PIDController
from bw_tools.controller.trapezoid_profile import Constraints


class ControllerType(Enum):
    RAMSETE = "ramsete"
    HOLONOMIC = "holonomic"
    NONHOLONOMIC = "nonholonomic"
    STRAFE = "strafe"


class GoToPoseCommand:
    def __init__(self) -> None:
        self.robot_state: Optional[Pose2d] = None

        self.cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
        self.goal_pose_pub = rospy.Publisher("go_to_pose_goal", PoseStamped, queue_size=10)
        self.goal_pose_sub = None
        self.controller_type = ControllerType(rospy.get_param("~go_to_pose/controller_type", "ramsete"))

        self.x_kP = rospy.get_param("~go_to_pose/x_kP", 1.0)
        self.x_kI = rospy.get_param("~go_to_pose/x_kI", 0.0)
        self.x_kD = rospy.get_param("~go_to_pose/x_kD", 0.0)

        self.y_kP = rospy.get_param("~go_to_pose/y_kP", 1.0)
        self.y_kI = rospy.get_param("~go_to_pose/y_kI", 0.0)
        self.y_kD = rospy.get_param("~go_to_pose/y_kD", 0.0)

        self.linear_min_vel = rospy.get_param("~go_to_pose/linear_min_vel", 0.0)
        self.linear_max_vel = rospy.get_param("~go_to_pose/linear_max_vel", 1.0)
        self.linear_max_accel = rospy.get_param("~go_to_pose/linear_max_accel", 3.0)

        self.theta_kP = rospy.get_param("~go_to_pose/theta_kP", 1.0)
        self.theta_kI = rospy.get_param("~go_to_pose/theta_kI", 0.0)
        self.theta_kD = rospy.get_param("~go_to_pose/theta_kD", 0.0)

        self.ramsete_b = rospy.get_param("~go_to_pose/ramsete_b", 2.0)
        self.ramsete_zeta = rospy.get_param("~go_to_pose/ramsete_zeta", 0.7)

        self.theta_min_vel = rospy.get_param("~go_to_pose/theta_min_vel", 0.0)
        self.theta_max_vel = rospy.get_param("~go_to_pose/theta_max_vel", 3.0)
        self.theta_max_accel = rospy.get_param("~go_to_pose/theta_max_accel", 6.0)

        self.strafe_angle_limit = rospy.get_param("~go_to_pose/strafe_angle_limit", math.pi / 2.0)

        self.loop_rate = rospy.get_param("~go_to_pose/loop_rate", 50.0)
        self.loop_period = 1.0 / self.loop_rate

        self.global_frame_id = rospy.get_param("~go_to_pose/global_frame", "map")
        self.robot_frame_id = rospy.get_param("~go_to_pose/robot_frame", "base_link")

        self.goal_pose_msg: Optional[PoseStamped] = None

        self.timeout_buffer = rospy.Duration(0.0)
        self.timeout = rospy.Duration(0.0)

        self.reference_linear_speed = 0.0  # Stored from last goal callback
        self.reference_angular_speed = 0.0  # Stored from last goal callback

        linear_constraints = Constraints(self.linear_max_vel, self.linear_max_accel)
        theta_constraints = Constraints(self.theta_max_vel, self.theta_max_accel)

        if self.controller_type == ControllerType.HOLONOMIC:
            self.controller = HolonomicDriveController(
                ProfiledPIDController(self.x_kP, self.x_kI, self.x_kD, linear_constraints, self.loop_period),
                ProfiledPIDController(self.y_kP, self.y_kI, self.y_kD, linear_constraints, self.loop_period),
                ProfiledPIDController(self.theta_kP, self.theta_kI, self.theta_kD, theta_constraints, self.loop_period)
            )
        elif self.controller_type == ControllerType.RAMSETE:
            self.controller = RamseteController(self.ramsete_b, self.ramsete_zeta)
        elif self.controller_type == ControllerType.NONHOLONOMIC:
            self.controller = NonHolonomicDriveController(
                PIDController(self.x_kP, self.x_kI, self.x_kD, self.loop_period),
                PIDController(self.y_kP, self.y_kI, self.y_kD, self.loop_period),
                ProfiledPIDController(self.theta_kP, self.theta_kI, self.theta_kD, theta_constraints, self.loop_period)
            )
        elif self.controller_type == ControllerType.STRAFE:
            self.controller = StrafeController(
                ProfiledPIDController(self.x_kP, self.x_kI, self.x_kD, linear_constraints, self.loop_period),
                ProfiledPIDController(self.y_kP, self.y_kI, self.y_kD, linear_constraints, self.loop_period),
                PIDController(self.theta_kP, self.theta_kI, self.theta_kD, self.loop_period),
                self.strafe_angle_limit
            )
        else:
            raise ValueError(f"Invalid controller type: {self.controller_type}")

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.goal_pose_sub = rospy.Subscriber("moving_goal", PoseStamped, self.moving_goal_callback, queue_size=10)

        self.action_server = actionlib.SimpleActionServer(
            "go_to_pose",
            GoToPoseAction,
            execute_cb=self.action_callback,
            auto_start=False
        )
        self.action_server.start()
        rospy.loginfo("go_to_pose is ready")

    def update_robot_state(self):
        try:
            transform = self.tf_buffer.lookup_transform(self.global_frame_id, self.robot_frame_id, rospy.Time(0), rospy.Duration(1.0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            return
        robot_pose = PoseStamped()
        robot_pose.header.frame_id = self.robot_frame_id
        robot_pose.pose.orientation.w = 1.0
        global_pose = tf2_geometry_msgs.do_transform_pose(robot_pose, transform)
        self.robot_state = Pose2d.from_ros_pose(global_pose.pose)

    def moving_goal_callback(self, msg):
        self.goal_pose_msg = msg
        self.timeout = self.compute_duration()
    
    def publish_velocity(self, velocity: Velocity):
        twist = Twist()
        twist.linear.x = velocity.x
        twist.linear.y = velocity.y
        twist.angular.z = velocity.theta
        self.cmd_vel_pub.publish(twist)
    
    def publish_state_feedback(self, current_pose2d: Pose2d, goal_pose2d: Pose2d, current_header: str, goal_header: str):
        feedback = GoToPoseFeedback()
        feedback.current_pose.header.frame_id = current_header
        feedback.current_pose.pose = current_pose2d.to_ros_pose()
        feedback.goal_pose.header.frame_id = goal_header
        feedback.goal_pose.pose = goal_pose2d.to_ros_pose()
        self.action_server.publish_feedback(feedback)

    def get_goal_pose(self):
        return Pose2d.from_ros_pose(self.goal_pose_msg.pose)
    
    def compute_duration(self, reference_linear_speed=None, reference_angular_speed=None):
        if reference_linear_speed is None:
            reference_linear_speed = self.reference_linear_speed
        if reference_angular_speed is None:
            reference_angular_speed = self.reference_angular_speed
        if self.goal_pose_msg is None:
            return self.timeout_buffer
        
        goal_pose2d = self.get_goal_pose()
        goal_pose2d.relative_to(self.robot_state)

        timeout = 10.0 * goal_pose2d.distance() / reference_linear_speed
        timeout += 10.0 * goal_pose2d.theta / reference_angular_speed
        return rospy.Duration(timeout) + self.timeout_buffer
    
    def goal_to_global_frame(self, goal_pose: PoseStamped, global_frame_id):
        try:
            transform = self.tf_buffer.lookup_transform(global_frame_id, goal_pose.header.frame_id, rospy.Time(0), rospy.Duration(1.0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            return None
        global_pose = tf2_geometry_msgs.do_transform_pose(goal_pose, transform)
        return global_pose

    def action_callback(self, goal: GoToPoseGoal):
        rospy.loginfo(f"Going to pose: {goal}")

        xy_tolerance: float = goal.xy_tolerance
        yaw_tolerance: float = goal.yaw_tolerance
        self.timeout_buffer: rospy.Duration = goal.timeout
        reference_linear_speed: float = goal.reference_linear_speed
        reference_angular_speed: float = goal.reference_angular_speed
        ignore_obstacles = goal.ignore_obstacles
        allow_reverse = goal.allow_reverse

        self.timeout = rospy.Duration(1.0)  # wait at least 1 second for a goal to come in

        self.controller.set_enabled(True)
        self.controller.set_tolerance(Pose2d(xy_tolerance, xy_tolerance, yaw_tolerance))

        self.goal_pose_msg = None

        rate = rospy.Rate(self.loop_rate)
        start_time = rospy.Time.now()
        current_time = rospy.Time.now()
        while current_time - start_time < self.timeout:
            rate.sleep()
            current_time = rospy.Time.now()
            self.update_robot_state()
            if self.robot_state is None:
                continue
            if self.goal_pose_msg is None:
                self.goal_pose_msg = self.goal_to_global_frame(goal.goal, self.global_frame_id)
                self.timeout = self.compute_duration(reference_linear_speed, reference_angular_speed)
                rospy.loginfo(f"Going to pose in {self.global_frame_id} frame: {self.goal_pose_msg}. Expected to take {self.timeout.to_sec()} seconds")

            goal_pose2d = self.get_goal_pose()

            goal_pose = PoseStamped()
            goal_pose.header.frame_id = self.global_frame_id
            goal_pose.pose = goal_pose2d.to_ros_pose()
            self.goal_pose_pub.publish(goal_pose)

            if self.controller_type == ControllerType.HOLONOMIC:
                velocity_command: Velocity = self.controller.calculate(
                    current_pose=self.robot_state, 
                    pose_ref=goal_pose2d, 
                    linear_velocity_ref=reference_linear_speed
                )
            elif self.controller_type == ControllerType.RAMSETE:
                velocity_command: Velocity = self.controller.calculate(
                    current_pose=self.robot_state, 
                    pose_ref=goal_pose2d, 
                    linear_velocity_ref=reference_linear_speed, 
                    angular_velocity_ref=reference_angular_speed
                )
            elif self.controller_type == ControllerType.NONHOLONOMIC:
                velocity_command: Velocity = self.controller.calculate(
                    current_pose=self.robot_state, 
                    pose_ref=goal_pose2d, 
                    linear_velocity_ref=reference_linear_speed, 
                    angular_velocity_ref=reference_angular_speed,
                    linear_min_velocity=self.linear_min_vel,
                    theta_min_velocity=self.theta_min_vel,
                    allow_reverse=allow_reverse
                )
            elif self.controller_type == ControllerType.STRAFE:
                velocity_command: Velocity = self.controller.calculate(
                    current_pose=self.robot_state, 
                    pose_ref=goal_pose2d, 
                    linear_velocity_ref=reference_linear_speed, 
                    angular_velocity_ref=reference_angular_speed,
                    linear_min_velocity=self.linear_min_vel,
                    theta_min_velocity=self.theta_min_vel,
                    allow_reverse=allow_reverse
                )
            else:
                raise ValueError(f"Invalid controller type: {self.controller_type}")
            rospy.loginfo(f"Robot pose: {self.robot_state}")
            rospy.loginfo(f"Goal pose: {goal_pose2d}")
            rospy.loginfo(f"Velocity command: {velocity_command}")
            self.publish_velocity(velocity_command)
            self.publish_state_feedback(
                self.robot_state,
                goal_pose2d,
                self.global_frame_id,
                self.goal_pose_msg.header.frame_id
            )
            if self.action_server.is_preempt_requested():
                rospy.loginfo(f"Cancelling go to pose")
                break
            if self.controller.at_reference():
                rospy.loginfo(f"Robot made it to goal. Current pose: {self.robot_state}. Goal pose: {goal_pose2d}")
                break
        
        if current_time - start_time > self.timeout:
            rospy.loginfo("Go to pose timed out")

        self.controller.set_enabled(False)
        if self.robot_state is None:
            success = False
            distance = 0.0
            angular_error = 0.0
            rospy.loginfo("Never received robot's position")
        else:
            distance = self.robot_state.distance(goal_pose2d)
            angular_error = abs(self.robot_state.theta - goal_pose2d.theta)

            success = (
                distance < xy_tolerance and
                angular_error < yaw_tolerance
            )
        result = GoToPoseResult(success)
        if result.success:
            rospy.loginfo("Return success for go to pose")
            self.action_server.set_succeeded(result)
        else:
            rospy.loginfo("Return failure for go to pose")
            if distance >= xy_tolerance:
                rospy.loginfo(f"Distance tolerance not met: {distance} >= {xy_tolerance}")
            if angular_error >= yaw_tolerance:
                rospy.loginfo(f"Angular tolerance not met: {angular_error} >= {yaw_tolerance}")
            self.action_server.set_aborted(result, "Failed to go to pose")
        self.robot_state = None
