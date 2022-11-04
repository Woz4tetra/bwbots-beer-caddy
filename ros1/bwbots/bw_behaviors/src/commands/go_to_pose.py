from typing import Optional
from enum import Enum

import rospy
import actionlib

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist

from nav_msgs.msg import Odometry

from bw_interfaces.msg import GoToPoseAction, GoToPoseGoal, GoToPoseFeedback, GoToPoseResult

from bw_tools.robot_state import Pose2d, Velocity
from bw_tools.controller.holonomic_drive_controller import HolonomicDriveController
from bw_tools.controller.ramsete_controller import RamseteController
from bw_tools.controller.profiled_pid import ProfiledPIDController
from bw_tools.controller.trapezoid_profile import Constraints


class ControllerType(Enum):
    RAMSETE = "ramsete"
    HOLONOMIC = "holonomic"


class GoToPoseCommand:
    def __init__(self) -> None:
        self.robot_state: Optional[Pose2d] = None
        self.cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
        self.odom_sub = None
        self.goal_pose_sub = None
        self.controller_type = ControllerType(rospy.get_param("~go_to_pose/controller_type", "ramsete"))

        self.x_kP = rospy.get_param("~go_to_pose/x_kP", 1.0)
        self.x_kI = rospy.get_param("~go_to_pose/x_kI", 0.0)
        self.x_kD = rospy.get_param("~go_to_pose/x_kD", 0.0)

        self.y_kP = rospy.get_param("~go_to_pose/y_kP", 1.0)
        self.y_kI = rospy.get_param("~go_to_pose/y_kI", 0.0)
        self.y_kD = rospy.get_param("~go_to_pose/y_kD", 0.0)

        self.linear_max_vel = rospy.get_param("~go_to_pose/linear_max_vel", 1.0)
        self.linear_max_accel = rospy.get_param("~go_to_pose/linear_max_accel", 3.0)

        self.theta_kP = rospy.get_param("~go_to_pose/theta_kP", 1.0)
        self.theta_kI = rospy.get_param("~go_to_pose/theta_kI", 0.0)
        self.theta_kD = rospy.get_param("~go_to_pose/theta_kD", 0.0)

        self.ramsete_b = rospy.get_param("~go_to_pose/ramsete_b", 2.0)
        self.ramsete_zeta = rospy.get_param("~go_to_pose/ramsete_zeta", 0.7)

        self.theta_max_vel = rospy.get_param("~go_to_pose/theta_max_vel", 3.0)
        self.theta_max_accel = rospy.get_param("~go_to_pose/theta_max_accel", 6.0)

        self.loop_rate = rospy.get_param("~go_to_pose/loop_rate", 50.0)
        self.loop_period = 1.0 / self.loop_rate

        self.odom_msg = Odometry()
        self.goal_pose_msg = PoseStamped()

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
        else:
            raise ValueError(f"Invalid controller type: {self.controller_type}")

        self.go_to_pose_server = actionlib.ActionServer(
            "go_to_pose",
            GoToPoseAction,
            self.go_to_pose_callback, 
            auto_start=False
        )
        self.go_to_pose_server.start()
        rospy.loginfo("go_to_pose is ready")

    def register_topics(self):
        self.odom_sub = rospy.Subscriber("odom", Odometry, self.odom_msg_callback, queue_size=10)
        self.goal_pose_sub = rospy.Subscriber("moving_goal", PoseStamped, self.odom_msg_callback, queue_size=10)
    
    def unregister_topics(self):
        if self.odom_sub is not None:
            self.odom_sub.unregister()
        self.robot_state = None

    def odom_msg_callback(self, msg):
        self.goal_pose_msg = msg
    
    def moving_goal_callback(self, msg):
        self.odom_msg = msg
        self.robot_state = Pose2d.from_ros_pose(self.odom_msg.pose.pose)
    
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
        self.go_to_pose_server.publish_feedback(feedback)

    def update_goal_pose(self):
        return Pose2d.from_ros_pose(self.goal_pose_msg.pose)

    def go_to_pose_callback(self, goal: GoToPoseGoal):
        self.register_topics()
        self.goal_pose_msg = goal.goal
        goal_pose2d = self.update_goal_pose()
        xy_tolerance: float = goal.xy_tolerance
        yaw_tolerance: float = goal.yaw_tolerance
        timeout = rospy.Duration(goal.timeout.data)
        reference_linear_speed: float = goal.reference_linear_speed
        reference_angular_speed: float = goal.reference_angular_speed
        # ignore_obstacles = goal.ignore_obstacles

        self.controller.set_enabled(True)
        self.controller.set_tolerance(Pose2d(xy_tolerance, xy_tolerance, yaw_tolerance))

        rate = rospy.Rate(self.loop_rate)
        start_time = rospy.Time.now()
        current_time = rospy.Time.now()
        while current_time - start_time < timeout:
            rate.sleep()
            current_time = rospy.Time.now()
            if self.robot_state is None:
                continue

            goal_pose2d = self.update_goal_pose()

            if self.controller_type == ControllerType.HOLONOMIC:
                velocity_command: Velocity = self.controller.calculate(
                    current_pose=self.robot_state, 
                    pose_ref=goal_pose2d, 
                    linear_velocity_ref_meters=reference_linear_speed
                )
            elif self.controller_type == ControllerType.RAMSETE:
                velocity_command: Velocity = self.controller.calculate(
                    current_pose=self.robot_state, 
                    pose_ref=goal_pose2d, 
                    linear_velocity_ref=reference_linear_speed, 
                    angular_velocity_ref=reference_angular_speed
                )
            else:
                raise ValueError(f"Invalid controller type: {self.controller_type}")
            self.publish_velocity(velocity_command)
            self.publish_state_feedback(
                self.robot_state,
                goal_pose2d,
                self.odom_msg.header.frame_id,
                self.goal_pose_msg.header.frame_id
            )
            if self.controller.at_reference():
                rospy.loginfo(f"Robot made it to goal. Current pose: {self.robot_state}. Goal pose: {goal_pose2d}")
                break

        self.unregister_topics()
        self.controller.set_enabled(False)
        success = (
            self.robot_state.distance(goal_pose2d) < xy_tolerance and
            abs(self.robot_state.theta - goal_pose2d.theta) < yaw_tolerance
        )
        result = GoToPoseResult(success)
        self.go_to_pose_server.publish_result(result)
        if result.success:
            self.go_to_pose_server.set_succeeded()
        else:
            self.go_to_pose_server.set_aborted()
