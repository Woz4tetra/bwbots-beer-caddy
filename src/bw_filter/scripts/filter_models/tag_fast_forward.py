import rospy
from copy import deepcopy
import numpy as np
from typing import Optional, List
from nav_msgs.msg import Odometry
from tf_conversions import transformations
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion

from .helpers import LANDMARK_COVARIANCE_INDICES
from .drive_kf_model import DriveKalmanModel


class TagFastForward:
    def __init__(
        self,
        dt: float,
        play_forward_buffer_size: int,
        lag_upper_threshold: float = 0.25,
    ) -> None:
        self.dt = dt
        self.play_forward_buffer_size = play_forward_buffer_size
        self.model = DriveKalmanModel(self.dt)
        self.lag_upper_threshold = lag_upper_threshold

        self.current_index = 0
        self.odom_messages: List[Odometry] = []

    @staticmethod
    def _get_rpy(quaternion: Quaternion):
        return transformations.euler_from_quaternion(
            (quaternion.x, quaternion.y, quaternion.z, quaternion.w)
        )

    def fast_forward(
        self, msg: PoseWithCovarianceStamped
    ) -> Optional[PoseWithCovarianceStamped]:
        start_time = msg.header.stamp.to_sec()
        now = rospy.Time.now().to_sec()
        lag = now - start_time
        if lag > self.lag_upper_threshold:
            rospy.logwarn(f"Lag is too high ({lag}), ignoring message")
            return None
        elif lag < 0.0:
            rospy.logwarn(
                f"Landmark has a timestamp in the future ({lag}), ignoring message"
            )
            return None
        num_samples = round(lag / self.dt)
        if num_samples > 0:
            self.model.reset(msg)
            self.current_index = 0
            for forwarded_time in np.linspace(start_time, now, num_samples):
                odom_msg = self._find_nearest_odom(forwarded_time)
                if odom_msg is not None:
                    self._update_odometry(odom_msg)
                self.model.predict()
            self.odom_messages = []

            roll, pitch, _ = self._get_rpy(msg.pose.pose.orientation)
            result = deepcopy(msg)
            pose = self.model.get_pose()
            result.pose.pose.position.x = pose.x
            result.pose.pose.position.y = pose.y
            result.pose.pose.orientation = Quaternion(
                *transformations.quaternion_from_euler(roll, pitch, pose.theta)
            )
            covariance = list(result.pose.covariance)
            new_covariance = self.model.get_covariance()
            for mat_index, msg_index in LANDMARK_COVARIANCE_INDICES.items():
                covariance[msg_index] = new_covariance[mat_index]
            result.pose.covariance = covariance
            return result
        else:
            return msg

    def _find_nearest_odom(self, timestamp: float) -> Optional[Odometry]:
        selected_msg = None
        index = self.current_index
        for index in range(self.current_index, len(self.odom_messages)):
            msg = self.odom_messages[index]
            odom_timestamp = msg.header.stamp.to_sec()
            time_error = odom_timestamp - timestamp
            if abs(time_error) < self.dt:
                selected_msg = msg
                break
            if time_error > 0.0:
                break
        self.current_index = index
        return selected_msg

    def _update_odometry(self, msg: Odometry) -> None:
        self.model.update_odometry(msg)

    def record_odometry(self, msg: Odometry) -> None:
        self.odom_messages.append(msg)
