import tf.transformations
from geometry_msgs.msg import PoseWithCovarianceStamped
from bw_tools.robot_state import Pose2d


def is_roll_pitch_reasonable(
    landmark: PoseWithCovarianceStamped,
    roll_pitch_threshold: float,
):
    landmark_quat = landmark.pose.pose.orientation
    roll, pitch, _ = tf.transformations.euler_from_quaternion(
        (landmark_quat.x, landmark_quat.y, landmark_quat.z, landmark_quat.w)
    )
    # check if the landmark is a reasonable roll and pitch
    return abs(roll) < roll_pitch_threshold and abs(pitch) < roll_pitch_threshold


def amcl_and_landmark_agree(
    amcl_pose: PoseWithCovarianceStamped,
    landmark: PoseWithCovarianceStamped,
    ground_distance_threshold: float,
    ground_angle_threshold: float,
) -> bool:
    landmark_pose2d = Pose2d.from_ros_pose(landmark.pose.pose)
    amcl_pose2d = Pose2d.from_ros_pose(amcl_pose.pose.pose)

    distance_delta = landmark_pose2d.distance(amcl_pose2d)
    angle_delta = abs(landmark_pose2d.theta - amcl_pose2d.theta)

    return (
        distance_delta < ground_distance_threshold
        and angle_delta < ground_angle_threshold
    )
