import copy
import rospy
import tf2_ros
import actionlib
import threading
import tf2_geometry_msgs

from typing import List

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseArray

from apriltag_ros.msg import AprilTagDetectionArray
from apriltag_ros.msg import AprilTagDetection

from bw_interfaces.msg import FindTagAction, FindTagGoal, FindTagFeedback, FindTagResult

from bw_tools.robot_state import Pose2d


class FindTagCommand:
    def __init__(self) -> None:
        self.timeout = rospy.Duration(rospy.get_param("~find_tag/timeout", 5.0))
        self.stale_tag_time = rospy.Duration(
            rospy.get_param("~find_tag/stale_tag_time", 5.0)
        )
        self.stored_frame = rospy.get_param("~find_tag/stored_frame", "map")
        self.stddev_limit = rospy.get_param("~find_tag/stddev_limit", 3.0)
        self.min_samples = rospy.get_param("~find_tag/min_samples", 3)

        self.detections = {}
        self.computed_poses = {}

        self.detections_lock = threading.Lock()

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.left_tag_sub = rospy.Subscriber(
            "/apriltag_left/tag_detections",
            AprilTagDetectionArray,
            self.tag_callback,
            queue_size=10,
        )
        self.right_tag_sub = rospy.Subscriber(
            "/apriltag_right/tag_detections",
            AprilTagDetectionArray,
            self.tag_callback,
            queue_size=10,
        )
        self.filtered_tag_pub = rospy.Publisher(
            "/apriltag/filtered_tag_detections", AprilTagDetectionArray, queue_size=10
        )
        self.tag_pose_pub = rospy.Publisher(
            "/apriltag/tag_poses", PoseArray, queue_size=10
        )
        self.filtered_tag_pose_pub = rospy.Publisher(
            "/apriltag/filtered_tag_poses", PoseArray, queue_size=10
        )

        self.action_server = actionlib.SimpleActionServer(
            "find_tag", FindTagAction, execute_cb=self.action_callback, auto_start=False
        )
        self.action_server.start()
        rospy.loginfo("find_tag is ready")

    def tag_callback(self, msg: AprilTagDetectionArray):
        with self.detections_lock:
            for tag in msg.detections:
                tag_id: List[int] = list(tag.id)
                frozen_tag_id = tuple(tag_id)
                if frozen_tag_id not in self.detections:
                    self.detections[frozen_tag_id] = []
                tf_msg = copy.deepcopy(tag)
                dest_tag_pose = self.get_tag_pose_in(self.stored_frame, tag)
                if dest_tag_pose is None:
                    return
                tf_msg.pose.header = dest_tag_pose.header
                tf_msg.pose.pose.pose = dest_tag_pose.pose
                self.detections[frozen_tag_id].append(tf_msg)

        self.update_computed_poses()

    def prune_detections(self):
        with self.detections_lock:
            for tag_id in self.detections:
                while (
                    len(self.detections[tag_id]) > 0
                    and rospy.Time.now() - self.detections[tag_id][0].pose.header.stamp
                    > self.stale_tag_time
                ):
                    self.detections[tag_id].pop(0)

    def update_computed_poses(self):
        self.prune_detections()
        with self.detections_lock:
            filtered_array_msg = AprilTagDetectionArray()
            pose_array = PoseArray()
            filtered_pose_array = PoseArray()

            for tag_id, messages in self.detections.items():
                self.computed_poses[tag_id] = (rospy.Time(0), Pose2d())
                filtered_msg = AprilTagDetection()
                pose2ds = []
                for msg in messages:
                    pose2ds.append(Pose2d.from_ros_pose(msg.pose.pose.pose))
                    pose_array.header = msg.pose.header
                    pose_array.poses.append(msg.pose.pose.pose)

                filtered_poses = []
                if len(pose2ds) > 0:
                    stddev_pose = Pose2d.stddev(pose2ds)
                    mean_pose = Pose2d.average(pose2ds)
                    lower_limit = mean_pose - stddev_pose * self.stddev_limit
                    upper_limit = mean_pose + stddev_pose * self.stddev_limit
                    for pose in pose2ds:
                        if (
                            pose.x < lower_limit.x
                            or pose.y < lower_limit.y
                            or pose.theta < lower_limit.theta
                        ):
                            continue
                        if (
                            pose.x > upper_limit.x
                            or pose.y > upper_limit.y
                            or pose.theta > upper_limit.theta
                        ):
                            continue
                        filtered_poses.append(pose)
                if len(filtered_poses) >= self.min_samples:
                    computed_pose = Pose2d.average(filtered_poses)
                    self.computed_poses[tag_id] = (rospy.Time.now(), computed_pose)

                    filtered_msg.id = tag_id
                    filtered_msg.pose.pose.pose = Pose2d.to_ros_pose(computed_pose)
                    filtered_msg.pose.header = messages[0].pose.header
                    filtered_array_msg.header = filtered_msg.pose.header
                    filtered_array_msg.detections.append(filtered_msg)
                    filtered_pose_array.header = filtered_msg.pose.header
                    filtered_pose_array.poses.append(filtered_msg.pose.pose.pose)

            if len(filtered_array_msg.detections) > 0:
                self.filtered_tag_pub.publish(filtered_array_msg)
                self.filtered_tag_pose_pub.publish(filtered_pose_array)
                self.tag_pose_pub.publish(pose_array)

    def get_tag_pose_in(
        self, destination_frame: str, detection_msg: AprilTagDetection
    ) -> PoseStamped:
        try:
            transform = self.tf_buffer.lookup_transform(
                destination_frame,
                detection_msg.pose.header.frame_id,
                rospy.Time(0),
                rospy.Duration(1.0),
            )
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as e:
            return None

        rotated_tag_pose = PoseStamped()
        rotated_tag_pose.header = detection_msg.pose.header
        rotated_tag_pose.pose = detection_msg.pose.pose.pose
        dest_tag_pose = tf2_geometry_msgs.do_transform_pose(rotated_tag_pose, transform)

        return dest_tag_pose

    def action_callback(self, goal: FindTagGoal):
        tag_id: List[int] = list(goal.tag_id)
        reference_frame: str = goal.reference_frame_id
        timeout: rospy.Duration = goal.timeout

        rospy.loginfo(f"Finding tag: {goal}")

        frozen_tag_id = tuple(tag_id)

        empty_pose = PoseStamped()
        empty_pose.header.frame_id = self.stored_frame
        result = FindTagResult(empty_pose, False)

        start_time = rospy.Time.now()
        current_time = rospy.Time.now()
        aborted = False
        if timeout.to_sec() <= 0.0:
            timeout = self.timeout
            rospy.loginfo(f"Using default timeout: {timeout}")
        while current_time - start_time < timeout:
            current_time = rospy.Time.now()
            if self.action_server.is_preempt_requested():
                aborted = True
                break

            for det_tag_id, messages in self.detections.items():
                if len(det_tag_id) != 0 and frozen_tag_id != det_tag_id:
                    continue
                for msg in messages:
                    feedback_pose = PoseStamped()
                    feedback_pose.header = msg.pose.header
                    feedback_pose.pose = msg.pose.pose.pose

                    feedback = FindTagFeedback()
                    feedback.sample = feedback_pose
                    feedback.num_samples = len(messages)
                    self.action_server.publish_feedback(feedback)

            self.update_computed_poses()
            if len(frozen_tag_id) == 0:
                poses = []
                for timestamp, pose2d in self.computed_poses.values():
                    poses.append(pose2d)
                computed_pose = Pose2d.average(poses)
                rospy.loginfo(f"Computed average of all tags: {computed_pose}")
            else:
                if frozen_tag_id not in self.computed_poses:
                    continue
                timestamp, computed_pose = self.computed_poses[frozen_tag_id]

            if current_time - timestamp > self.stale_tag_time:
                continue
            else:
                rospy.loginfo(
                    f"Computed average of tag {frozen_tag_id}: {computed_pose}"
                )

            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = self.stored_frame
            pose_stamped.pose = computed_pose.to_ros_pose()

            if self.stored_frame != reference_frame:
                rospy.loginfo(
                    f"Transforming tag from {self.stored_frame} -> {reference_frame}"
                )
                try:
                    transform = self.tf_buffer.lookup_transform(
                        reference_frame,
                        self.stored_frame,
                        rospy.Time(0),
                        rospy.Duration(1.0),
                    )
                except (
                    tf2_ros.LookupException,
                    tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException,
                ) as e:
                    rospy.logwarn(f"Failed to transform tag: {e}")
                    continue
                tf_pose = tf2_geometry_msgs.do_transform_pose(pose_stamped, transform)
                result.pose = tf_pose
            else:
                result.pose = pose_stamped
            result.success = True
            break

        if aborted:
            self.action_server.set_aborted(result, "Interrupted while finding tag")
        else:
            if result.success:
                rospy.loginfo(f"Found tag: {goal}")
            else:
                rospy.loginfo(f"Failed to find tag: {goal}")
            self.action_server.set_succeeded(result)
