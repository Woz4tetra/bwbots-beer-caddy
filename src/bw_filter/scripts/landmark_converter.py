#!/usr/bin/env python3
import math
from typing import List, Optional

import numpy as np
import rospy
import tf2_geometry_msgs
import tf2_ros
import tf.transformations
from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped

from bw_tools.robot_state import SimpleFilter
from bw_tools.transforms import lookup_transform
from bw_tools.typing.basic import get_param


class LandmarkConverter:
    def __init__(self) -> None:
        rospy.init_node("landmark_converter")

        self.base_frame = get_param("~base_frame", "base_tilt_link")
        self.map_frame = get_param("~map_frame", "map")
        self.field_frame = get_param("~field_frame", "field")
        self.max_tag_distance = get_param("~max_tag_distance", 2.0)
        self.time_covariance_filter_k = get_param("~time_covariance_filter_k", 0.5)
        self.landmark_ids = frozenset(get_param("~landmark_ids", [0]))
        base_covariance = get_param("~covariance", np.eye(6, dtype=np.float64).flatten().tolist())
        assert len(base_covariance) == 36, f"Invalid covariance. Length is {len(base_covariance)}: {base_covariance}"
        self.base_covariance = np.array(base_covariance).reshape((6, 6))

        self.landmark = PoseWithCovarianceStamped()
        self.landmark.header.frame_id = self.base_frame

        self.prev_landmark = PoseStamped()

        self.time_covariance_filter = SimpleFilter(self.time_covariance_filter_k)

        self.buffer = tf2_ros.Buffer()
        self.transform_listener = tf2_ros.TransformListener(self.buffer)

        self.tag_sub = rospy.Subscriber("tag_detections", AprilTagDetectionArray, self.tags_callback, queue_size=10)
        self.landmark_pub = rospy.Publisher("landmark", PoseWithCovarianceStamped, queue_size=10)

    def tags_callback(self, msg: AprilTagDetectionArray) -> None:
        assert msg.detections is not None
        landmark_pose: Optional[PoseStamped] = None
        individual_measurements = []
        for detection in msg.detections:
            ids = frozenset(detection.id)
            detection_pose = PoseStamped()
            detection_pose.header = detection.pose.header
            detection_pose.pose = detection.pose.pose.pose
            if self.landmark_ids == ids:
                landmark_pose = detection_pose
            elif len(ids) == 1 and next(iter(ids)) in self.landmark_ids:
                individual_measurements.append(detection_pose)
        if landmark_pose is not None:
            # self.publish_landmark_from_tf()
            if self.should_publish(individual_measurements):
                self.publish_inverted_landmark(landmark_pose)
            if len(individual_measurements) > 0:
                self.update_covariance(individual_measurements, landmark_pose)

    def should_publish(self, tag_poses: List[PoseStamped]) -> bool:
        """
        Should publish if at least one tag is closer than the threshold distance
        """
        for tag_pose in tag_poses:
            distance = self.get_distance(tag_pose)
            if distance < self.max_tag_distance:
                return True
        return False

    def get_distance(self, tag_pose: PoseStamped) -> float:
        x = tag_pose.pose.position.x
        y = tag_pose.pose.position.y
        z = tag_pose.pose.position.z
        return math.sqrt(x * x + y * y + z * z)

    def publish_landmark_from_tf(self) -> None:
        transform = lookup_transform(self.buffer, self.field_frame, self.base_frame)
        zero_pose = PoseStamped()
        zero_pose.header.frame_id = self.base_frame
        zero_pose.pose.orientation.w = 1.0
        base_in_field = tf2_geometry_msgs.do_transform_pose(zero_pose, transform)
        self.landmark.header = base_in_field.header
        self.landmark.pose.pose = base_in_field.pose
        self.landmark_pub.publish(self.landmark)

    def publish_inverted_landmark(self, pose: PoseStamped) -> None:
        inverted_pose = self.invert_transform_pose(pose)
        if inverted_pose is None:
            return
        self.landmark.header = inverted_pose.header
        self.landmark.pose.pose = inverted_pose.pose
        self.landmark_pub.publish(self.landmark)

    def invert_transform_pose(self, pose: PoseStamped) -> Optional[PoseStamped]:
        transform = lookup_transform(self.buffer, self.base_frame, pose.header.frame_id)
        if transform is None:
            return None
        landmark_in_base = tf2_geometry_msgs.do_transform_pose(pose, transform)
        translation = (
            landmark_in_base.pose.position.x,
            landmark_in_base.pose.position.y,
            landmark_in_base.pose.position.z,
        )
        rotation = (
            landmark_in_base.pose.orientation.x,
            landmark_in_base.pose.orientation.y,
            landmark_in_base.pose.orientation.z,
            landmark_in_base.pose.orientation.w,
        )

        inverse_mat = tf.transformations.inverse_matrix(tf.transformations.translation_matrix(translation))

        inverse_mat = tf.transformations.concatenate_matrices(
            tf.transformations.inverse_matrix(tf.transformations.quaternion_matrix(rotation)),
            inverse_mat,
        )

        forward_translation = tf.transformations.translation_from_matrix(inverse_mat)
        forward_rotation = tf.transformations.quaternion_from_matrix(inverse_mat)
        inverse_pose = PoseStamped()
        inverse_pose.header.frame_id = self.map_frame
        inverse_pose.header.stamp = pose.header.stamp
        inverse_pose.pose.position.x = forward_translation[0]
        inverse_pose.pose.position.y = forward_translation[1]
        inverse_pose.pose.position.z = forward_translation[2]
        inverse_pose.pose.orientation.x = forward_rotation[0]
        inverse_pose.pose.orientation.y = forward_rotation[1]
        inverse_pose.pose.orientation.z = forward_rotation[2]
        inverse_pose.pose.orientation.w = forward_rotation[3]
        return inverse_pose

    def num_tags_covariance_scale(self, num_tags: int) -> float:
        if num_tags == 0:
            raise ValueError("Can't compute covariance with no measurements!")
        # 1 -> 1
        # 2 -> 0.5
        # 3 -> 0.25
        # 4 -> 0.125
        scale = 1.0 / (2 ** (num_tags - 1))
        return scale

    def pose_distance_covariance_scale(self, distance: float) -> float:
        if distance < 0.5:
            return 10.0
        return 0.25 * distance**2.0

    def get_pose_distance(self, pose: PoseStamped) -> float:
        return float(
            np.linalg.norm(
                [
                    pose.pose.position.x,
                    pose.pose.position.y,
                    pose.pose.position.z,
                ]
            )
        )

    def pose_stamped_to_vector(self, pose: PoseStamped) -> np.ndarray:
        return np.array(
            [
                pose.pose.position.x,
                pose.pose.position.y,
                pose.pose.position.z,
            ]
        )

    def delta_pose_covariance_scale(self, current_pose: PoseStamped, prev_pose: PoseStamped) -> float:
        distance = np.linalg.norm(self.pose_stamped_to_vector(current_pose) - self.pose_stamped_to_vector(prev_pose))
        scale = 4 * float(distance) + 1.0
        return scale

    def time_covariance_scale(self, current_pose: PoseStamped, prev_pose: PoseStamped) -> float:
        time_delta = (current_pose.header.stamp - prev_pose.header.stamp).to_sec()
        time_delta = max(time_delta, 5.0)
        filtered_delta = self.time_covariance_filter.update(time_delta)
        if filtered_delta < time_delta:
            time_delta = filtered_delta
        return time_delta * 2.0

    def update_covariance(self, tag_poses: List[PoseStamped], overall_pose: PoseStamped) -> None:
        distances = [self.get_pose_distance(pose) for pose in tag_poses]
        aggregate_distance = float(np.median(distances))

        covariance = self.base_covariance * self.num_tags_covariance_scale(len(tag_poses))
        covariance *= self.pose_distance_covariance_scale(aggregate_distance)

        covariance *= self.delta_pose_covariance_scale(overall_pose, self.prev_landmark)
        covariance *= self.time_covariance_scale(overall_pose, self.prev_landmark)

        self.prev_landmark = overall_pose
        self.landmark.pose.covariance = covariance.flatten().tolist()

    def run(self) -> None:
        rospy.spin()


if __name__ == "__main__":
    node = LandmarkConverter()
    node.run()
