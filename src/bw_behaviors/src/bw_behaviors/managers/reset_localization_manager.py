import math

import rospy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped

from bw_interfaces.msg import Waypoint


class ResetLocalizationManager:
    def __init__(
        self, set_pose_topic="/initialpose", reset_x_std=0.1, reset_y_std=0.1, reset_theta_std_degrees=25.0
    ) -> None:
        self.reset_x_cov = reset_x_std * reset_x_std
        self.reset_y_cov = reset_y_std * reset_y_std
        reset_theta_std = math.radians(reset_theta_std_degrees)
        self.reset_theta_cov = reset_theta_std * reset_theta_std
        self.set_pose_pub = rospy.Publisher(set_pose_topic, PoseWithCovarianceStamped, queue_size=5)

    def reset_to_waypoint(self, waypoint: Waypoint) -> None:
        pose = PoseStamped()
        pose.header = waypoint.header
        pose.pose = waypoint.pose
        self.reset_to_pose(pose)

    def reset_to_pose(self, pose: PoseStamped) -> None:
        robot_estimate = PoseWithCovarianceStamped()
        robot_estimate.header = pose.header
        robot_estimate.pose.pose = pose.pose
        robot_estimate.pose.covariance[0] = self.reset_x_cov  # type: ignore
        robot_estimate.pose.covariance[7] = self.reset_y_cov  # type: ignore
        robot_estimate.pose.covariance[35] = self.reset_theta_cov  # type: ignore
        rospy.loginfo(f"Setting robot pose to {robot_estimate}")
        self.set_pose_pub.publish(robot_estimate)
