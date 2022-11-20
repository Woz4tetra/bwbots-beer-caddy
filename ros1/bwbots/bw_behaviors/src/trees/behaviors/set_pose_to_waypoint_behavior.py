import math
import rospy
import py_trees

from geometry_msgs.msg import PoseWithCovarianceStamped

from trees.managers.waypoint_manager import WaypointManager


class SetPoseToWaypointBehavior(py_trees.behaviour.Behaviour):
    def __init__(
            self,
            waypoint_name,
            waypoint_manager: WaypointManager,
            set_pose_topic="/initialpose",
            reset_x_std=0.1,
            reset_y_std=0.1,
            reset_theta_std_degrees=25.0):
        super().__init__("Set pose to waypoint")
        self.waypoint_name = waypoint_name
        self.waypoint_manager = waypoint_manager
        self.set_pose_pub = rospy.Publisher(set_pose_topic, PoseWithCovarianceStamped, queue_size=5)
        
        self.reset_x_cov = reset_x_std * reset_x_std
        self.reset_y_cov = reset_y_std * reset_y_std
        reset_theta_std = math.radians(reset_theta_std_degrees)
        self.reset_theta_cov = reset_theta_std * reset_theta_std

    def update(self):
        waypoint_array = self.waypoint_manager.get_waypoint(self.waypoint_name)
        if waypoint_array is None:
            return py_trees.Status.FAILURE
        waypoint = waypoint_array.waypoints[0]
        robot_estimate = PoseWithCovarianceStamped()
        robot_estimate.header = waypoint.header
        robot_estimate.pose.pose = waypoint.pose
        robot_estimate.pose.covariance[0] = self.reset_x_cov
        robot_estimate.pose.covariance[7] = self.reset_y_cov
        robot_estimate.pose.covariance[35] = self.reset_theta_cov
        rospy.loginfo(f"Setting robot pose to {robot_estimate}")
        self.set_pose_pub.publish(robot_estimate)
        return py_trees.Status.SUCCESS
