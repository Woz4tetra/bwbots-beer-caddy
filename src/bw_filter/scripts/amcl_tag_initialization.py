#!/usr/bin/env python3
import numpy as np
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from helpers import amcl_and_landmark_agree, is_roll_pitch_reasonable
from bw_tools.robot_state import Velocity, Pose2d


class AmclTagInitialization:
    def __init__(self) -> None:
        rospy.init_node("amcl_tag_initialization")

        self.linear_velocity_threshold = rospy.get_param(
            "~linear_velocity_threshold", 0.1
        )
        self.angular_velocity_threshold = rospy.get_param(
            "~angular_velocity_threshold", 0.1
        )
        self.z_threshold = rospy.get_param("~z_threshold", 0.75)
        self.roll_pitch_threshold = rospy.get_param("~roll_pitch_threshold", 0.2)
        self.ground_distance_threshold = rospy.get_param(
            "~ground_distance_threshold", 0.5
        )
        self.ground_angle_threshold = rospy.get_param("~ground_angle_threshold", 0.5)

        self.initial_cov_xx = rospy.get_param("/amcl/initial_cov_xx", 0.25)
        self.initial_cov_yy = rospy.get_param("/amcl/initial_cov_yy", 0.25)
        self.initial_cov_aa = rospy.get_param("/amcl/initial_cov_aa", 0.06854)

        # 0.0 == only reset once
        self.cooldown_time = rospy.Duration(rospy.get_param("~cooldown_time", 0.0))  # type: ignore

        self.stale_measurement_time = rospy.Duration(
            rospy.get_param("~stale_measurement_time", 0.5)  # type: ignore
        )

        self.last_reset_time = rospy.Time()

        self.robot_velocity_timestamp = rospy.Time()
        self.robot_velocity = Velocity()
        self.amcl_pose = PoseWithCovarianceStamped()

        self.landmark_sub = rospy.Subscriber(
            "landmark",
            PoseWithCovarianceStamped,
            self.landmark_callback,
            queue_size=10,
        )
        self.amcl_pose_sub = rospy.Subscriber(
            "amcl_pose",
            PoseWithCovarianceStamped,
            self.amcl_pose_callback,
            queue_size=10,
        )
        self.odom_sub = rospy.Subscriber(
            "odom", Odometry, self.odom_callback, queue_size=10
        )
        self.initial_pose_pub = rospy.Publisher(
            "initialpose", PoseWithCovarianceStamped, queue_size=10
        )

    def landmark_callback(self, msg: PoseWithCovarianceStamped) -> None:
        now = rospy.Time.now()

        if self.cooldown_time.to_sec() == 0.0:
            # if should only reset once, check if a reset has occurred
            if self.last_reset_time != rospy.Time():
                return
        elif now - self.last_reset_time < self.cooldown_time:
            # if a reset should happen only after a cool down, check if the timer has been exceeded
            return

        if len(self.amcl_pose.header.frame_id) == 0:
            # check if the amcl pose has been set
            return

        if now - self.robot_velocity_timestamp > self.stale_measurement_time:
            # check if odometry is too old
            rospy.logwarn(
                f"Odometry velocity is stale. It is {(now - self.robot_velocity_timestamp).to_sec()} seconds old."
            )
            return

        if self.amcl_pose.header.frame_id != msg.header.frame_id:
            # check if the landmark frame ID matches AMCL
            rospy.logwarn(
                "AMCL pose frame ID doesn't match landmark: "
                f"{self.amcl_pose.header.frame_id} != {msg.header.frame_id}"
            )
            return

        if (
            self.linear_velocity_threshold > 0.0
            and self.robot_velocity.magnitude() > self.linear_velocity_threshold
        ):
            # check if the robot is moving below the linear threshold
            rospy.loginfo("Rejecting landmark. Linear velocity is too high")
            return

        if (
            self.angular_velocity_threshold > 0.0
            and abs(self.robot_velocity.theta) > self.angular_velocity_threshold
        ):
            # check if the robot is moving below the angular threshold
            rospy.loginfo("Rejecting landmark. Angular velocity is too high")
            return

        if abs(msg.pose.pose.position.z) > self.z_threshold:
            # check if the landmark is a reasonable Z height
            rospy.loginfo("Rejecting landmark. Z height is too high")
            return

        if not is_roll_pitch_reasonable(msg, self.roll_pitch_threshold):
            rospy.loginfo("Rejecting landmark. Roll or pitch is too high")
            return

        if not amcl_and_landmark_agree(
            self.amcl_pose,
            msg,
            self.ground_distance_threshold,
            self.ground_angle_threshold,
        ):
            # reset AMCL pose if landmark doesn't agree within a threshold
            landmark_pose2d = Pose2d.from_ros_pose(msg.pose.pose)
            self.reset_amcl_pose(msg.header.frame_id, landmark_pose2d)

    def amcl_pose_callback(self, msg: PoseWithCovarianceStamped) -> None:
        self.amcl_pose = msg

    def odom_callback(self, msg: Odometry) -> None:
        self.robot_velocity = Velocity.from_ros_twist(msg.twist.twist)
        self.robot_velocity_timestamp = rospy.Time.now()

    def reset_amcl_pose(self, frame_id: str, pose2d: Pose2d) -> None:
        rospy.loginfo(f"Reset AMCL to {pose2d}")
        reset_pose = PoseWithCovarianceStamped()
        reset_pose.header.frame_id = frame_id
        reset_pose.pose.pose = pose2d.to_ros_pose()
        mat = np.eye(6)
        mat[0, 0] = self.initial_cov_xx
        mat[1, 1] = self.initial_cov_yy
        mat[5, 5] = self.initial_cov_aa
        reset_pose.pose.covariance = mat.flatten().tolist()
        self.initial_pose_pub.publish(reset_pose)
        self.last_reset_time = rospy.Time.now()

    def run(self) -> None:
        rospy.spin()


if __name__ == "__main__":
    node = AmclTagInitialization()
    node.run()
