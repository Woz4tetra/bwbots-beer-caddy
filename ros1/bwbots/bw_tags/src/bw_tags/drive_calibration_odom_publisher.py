#!/usr/bin/env python3
from typing import Dict, Optional, Tuple

import rospy
import tf2_ros
import tf2_geometry_msgs

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from bw_tools.robot_state import Pose2d, Velocity


class DriveCalibrationOdomPublisher:
    def __init__(self):
        self.name = "drive_calibration_odom_publisher"
        rospy.init_node(
            self.name
            # disable_signals=True
            # log_level=rospy.DEBUG
        )
        # rospy.on_shutdown(self.shutdown_hook)
        self.rate = rospy.get_param("~rate", 15.0)
        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        self.tag_odom_pub = rospy.Publisher("tag_odom", Odometry, queue_size=5)
        self.amcl_odom_pub = rospy.Publisher("amcl_odom", Odometry, queue_size=5)
        
        self.pose_table: Dict[str, Tuple[float, Pose2d]] = {}

        rospy.loginfo("%s init complete" % self.name)

    def tf_to_odom(self, parent_frame, child_frame) -> Optional[Odometry]:
        now = rospy.Time.now()
        try:
            transform = self.tf_buffer.lookup_transform(
                parent_frame,
                child_frame,
                rospy.Time(0),
                rospy.Duration(1.0)
            )
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn("Failed to look up %s to %s. %s" % (parent_frame, child_frame, e))
            return None

        key = f"{parent_frame}-{child_frame}"

        identity_pose = PoseStamped()
        identity_pose.pose.orientation.w = 1.0
        identity_pose.header.frame_id = child_frame
        parent_pose = tf2_geometry_msgs.do_transform_pose(identity_pose, transform)
        parent_pose2d = Pose2d.from_ros_pose(parent_pose.pose)

        timestamp = now.to_sec()
        if key not in self.pose_table:
            self.pose_table[key] = timestamp, parent_pose2d
        prev_timestamp, prev_pose2d = self.pose_table[key]
        self.pose_table[key] = timestamp, parent_pose2d
        dt = timestamp - prev_timestamp
        if dt == 0.0:
            return None
        velocity = Velocity.from_poses(prev_pose2d, parent_pose2d, dt)
        
        msg = Odometry()
        msg.header.stamp = now
        msg.header.frame_id = parent_frame
        msg.child_frame_id = child_frame
        msg.pose.pose = parent_pose2d.to_ros_pose()
        msg.twist.twist = velocity.to_ros_twist()
        return msg

    def run(self):
        rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            tag_odom = self.tf_to_odom("floor", "drive_calibration_base_link")
            if tag_odom:
                self.tag_odom_pub.publish(tag_odom)
                
            amcl_odom = self.tf_to_odom("map", "base_link")
            if amcl_odom:
                self.amcl_odom_pub.publish(amcl_odom)
            rate.sleep()


if __name__ == "__main__":
    node = DriveCalibrationOdomPublisher()
    node.run()
