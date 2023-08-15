#!/usr/bin/env python3
from tf_conversions import transformations
import numpy as np
import tf2_ros
import rospy
from threading import Lock

from nav_msgs.msg import Odometry
from geometry_msgs.msg import (
    PoseWithCovarianceStamped,
    Quaternion,
    TransformStamped,
    PoseStamped,
    Vector3,
    Pose,
    Point,
)

from filter_models import TagFastForward
from filter_models import DriveKalmanModel as FilterModel
from helpers import amcl_and_landmark_agree, is_roll_pitch_reasonable
from bw_tools.robot_state import Pose2d, Velocity


class BwFilter:
    def __init__(self) -> None:
        rospy.init_node("bw_filter")
        self.map_frame = rospy.get_param("~map_frame", "map")
        self.odom_frame = ""
        self.base_frame = ""

        self.play_forward_buffer_size = rospy.get_param("~play_forward_buffer_size", 20)
        self.tag_fast_forward_sample_dt = rospy.get_param(
            "~tag_fast_forward_sample_dt", 0.05
        )
        self.update_rate = rospy.get_param("~update_rate", 50.0)
        self.roll_pitch_threshold = rospy.get_param("~roll_pitch_threshold", 0.2)
        self.ground_distance_threshold = rospy.get_param(
            "~ground_distance_threshold", 0.5
        )
        self.ground_angle_threshold = rospy.get_param("~ground_angle_threshold", 0.5)

        self.prev_odom = Odometry()

        self.model = FilterModel(1.0 / self.update_rate)
        self.model_lock = Lock()

        self.fast_forwarder = TagFastForward(
            self.tag_fast_forward_sample_dt, self.play_forward_buffer_size
        )

        self.amcl_pose = PoseWithCovarianceStamped()
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        self.odom_sub = rospy.Subscriber(
            "odom", Odometry, self.odom_callback, queue_size=10
        )
        self.landmark_sub = rospy.Subscriber(
            "landmark", PoseWithCovarianceStamped, self.landmark_callback, queue_size=10
        )
        self.reset_sub = rospy.Subscriber(
            "reset_pose", PoseWithCovarianceStamped, self.reset_callback, queue_size=10
        )
        self.amcl_pose_sub = rospy.Subscriber(
            "amcl_pose",
            PoseWithCovarianceStamped,
            self.amcl_pose_callback,
            queue_size=10,
        )
        self.initial_pose_pub = rospy.Publisher(
            "initialpose", PoseWithCovarianceStamped, queue_size=10
        )
        self.forwarded_landmark_pub = rospy.Publisher(
            "landmark/forwarded", PoseWithCovarianceStamped, queue_size=10
        )
        self.filter_state_pub = rospy.Publisher("filter/state", Odometry, queue_size=10)
        self.filter_pose_pub = rospy.Publisher(
            "filter/pose", PoseWithCovarianceStamped, queue_size=10
        )

    def get_last_pose(self) -> Pose:
        return self.prev_odom.pose.pose

    def odom_callback(self, msg: Odometry) -> None:
        if len(self.base_frame) == 0:
            self.base_frame = msg.child_frame_id
        if len(self.odom_frame) == 0:
            self.odom_frame = msg.header.frame_id
        if msg.child_frame_id != self.base_frame:
            raise ValueError(
                "Odometry child frame is inconsistent "
                f"{msg.child_frame_id} != {self.base_frame}"
            )
        if msg.header.frame_id != self.odom_frame:
            raise ValueError(
                "Odometry frame is inconsistent "
                f"{msg.header.frame_id} != {self.odom_frame}"
            )

        self.prev_odom = msg
        with self.model_lock:
            self.model.update_odometry(msg)
            self.fast_forwarder.record_odometry(msg)

    def publish_transform(self, pose: PoseStamped, child_frame: str) -> None:
        transform = TransformStamped()
        transform.header.stamp = rospy.Time.now()
        transform.header.frame_id = pose.header.frame_id
        transform.child_frame_id = child_frame
        transform.transform.translation = Vector3(
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z
        )
        transform.transform.rotation = pose.pose.orientation
        self.tf_broadcaster.sendTransform(transform)

    def publish_filter_state(
        self, pose: Pose2d, velocity: Velocity, covariance: np.ndarray
    ) -> None:
        state_msg = Odometry()
        state_msg.header.stamp = rospy.Time.now()
        state_msg.header.frame_id = self.map_frame
        state_msg.child_frame_id = self.base_frame
        state_msg.pose.pose = pose.to_ros_pose()
        state_msg.twist.twist = velocity.to_ros_twist()

        pose_covariance = np.zeros((6, 6))
        pose_covariance[0, 0] = covariance[0, 0]
        pose_covariance[1, 1] = covariance[1, 1]
        pose_covariance[5, 5] = covariance[2, 2]
        state_msg.pose.covariance = pose_covariance.flatten().tolist()

        twist_covariance = np.zeros((6, 6))
        twist_covariance[0, 0] = covariance[3, 3]
        twist_covariance[1, 1] = covariance[4, 4]
        twist_covariance[5, 5] = covariance[5, 5]
        state_msg.twist.covariance = twist_covariance.flatten().tolist()

        self.filter_state_pub.publish(state_msg)

        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header = state_msg.header
        pose_msg.pose = state_msg.pose
        self.filter_pose_pub.publish(pose_msg)

    def get_map_to_odom(self, map_to_base_pose: Pose, odom_to_base_pose: Pose) -> Pose:
        base2map = self.get_reverse_transform_mat(map_to_base_pose)
        odom2base = self.get_forward_transform_mat(odom_to_base_pose)
        map2odom = transformations.inverse_matrix(odom2base @ base2map)
        return self.get_pose_from_mat(map2odom)

    def get_pose_from_mat(self, mat: np.ndarray) -> Pose:
        pose = Pose()
        pose.position = Point(*transformations.translation_from_matrix(mat))
        pose.orientation = Quaternion(*transformations.quaternion_from_matrix(mat))
        return pose

    def get_forward_transform_mat(self, pose: Pose) -> np.ndarray:
        translation = (
            pose.position.x,
            pose.position.y,
            pose.position.z,
        )
        rotation = (
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )

        mat = np.array(
            transformations.concatenate_matrices(
                transformations.translation_matrix(translation),
                transformations.quaternion_matrix(rotation),
            )
        )
        return mat

    def get_reverse_transform_mat(self, pose: Pose) -> np.ndarray:
        return transformations.inverse_matrix(self.get_forward_transform_mat(pose))

    def landmark_callback(self, msg: PoseWithCovarianceStamped) -> None:
        if len(self.amcl_pose.header.frame_id) == 0:
            rospy.logwarn("AMCL pose not set. Not updating landmark")
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
            rospy.logwarn("AMCL pose do not agree. Not updating landmark")
            return
        forwarded = self.fast_forwarder.fast_forward(msg)
        if forwarded is not None:
            self.forwarded_landmark_pub.publish(forwarded)
            with self.model_lock:
                self.model.update_landmark(forwarded)

    def reset_callback(self, msg: PoseWithCovarianceStamped) -> None:
        self.model.reset(msg)
        self.initial_pose_pub.publish(msg)

    def amcl_pose_callback(self, msg: PoseWithCovarianceStamped) -> None:
        self.amcl_pose = msg
        with self.model_lock:
            self.model.update_landmark(msg)

    def run(self) -> None:
        rate = rospy.Rate(self.update_rate)
        while not rospy.is_shutdown():
            try:
                rate.sleep()
            except rospy.exceptions.ROSTimeMovedBackwardsException:
                continue

            if len(self.odom_frame) == 0 and len(self.base_frame) == 0:
                continue
            with self.model_lock:
                self.model.predict()
                self.publish_filter_state(
                    self.model.get_pose(),
                    self.model.get_velocity(),
                    self.model.get_covariance(),
                )
                global_pose = self.model.get_pose()
            odom_pose = self.get_last_pose()
            if odom_pose is not None:
                tf_pose = PoseStamped()
                tf_pose.header.frame_id = self.map_frame
                tf_pose.pose = self.get_map_to_odom(
                    global_pose.to_ros_pose(), odom_pose
                )
                self.publish_transform(tf_pose, self.odom_frame)


def main():
    node = BwFilter()
    node.run()


if __name__ == "__main__":
    main()
