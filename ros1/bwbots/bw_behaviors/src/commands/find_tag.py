from typing import List
import rospy
import tf2_ros
import actionlib
import tf2_geometry_msgs

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TransformStamped

from apriltag_ros.msg import AprilTagDetectionArray
from apriltag_ros.msg import AprilTagDetection

from bw_interfaces.msg import FindTagAction, FindTagGoal, FindTagFeedback, FindTagResult

from bw_tools.robot_state import Pose2d


class FindTagCommand:
    def __init__(self) -> None:
        self.num_samples = rospy.get_param("~find_tag/num_samples", 10)
        self.timeout = rospy.Duration(rospy.get_param("~find_tag/timeout", 5.0))
        self.stale_tag_time = rospy.Duration(rospy.get_param("~find_tag/stale_tag_time", 5.0))

        self.tag_ring = [None for _ in range(self.num_samples)]
        self.ring_index = 0
        self.left_tag_sub = rospy.Subscriber("/apriltag_left/tag_detections", AprilTagDetectionArray, self.tag_callback, queue_size=10)
        self.right_tag_sub = rospy.Subscriber("/apriltag_right/tag_detections", AprilTagDetectionArray, self.tag_callback, queue_size=10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.action_server = actionlib.SimpleActionServer(
            "find_tag",
            FindTagAction,
            execute_cb=self.action_callback, 
            auto_start=False
        )
        self.action_server.start()
        rospy.loginfo("find_tag is ready")
    
    def tag_callback(self, msg):
        for detection in msg.detections:
            self.add_detection(detection)

    def add_detection(self, detection: AprilTagDetection):
        self.tag_ring[self.ring_index] = detection
        self.ring_index = (self.ring_index + 1) % len(self.tag_ring)

    def get_tag_pose_in(self, destination_frame: str, detection_msg: AprilTagDetection) -> PoseStamped:
        try:
            transform = self.tf_buffer.lookup_transform(destination_frame, detection_msg.pose.header.frame_id, rospy.Time(0), rospy.Duration(1.0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            return None

        rotated_tag_pose = PoseStamped()
        rotated_tag_pose.header = detection_msg.pose.header
        rotated_tag_pose.pose = detection_msg.pose.pose.pose
        dest_tag_pose = tf2_geometry_msgs.do_transform_pose(rotated_tag_pose, transform)

        return dest_tag_pose
        

    def action_callback(self, goal: FindTagGoal):
        tag_id: List[int] = list(goal.tag_id)
        reference_frame: str = goal.reference_frame_id

        rospy.loginfo(f"Finding tag: {goal}")

        tag_id.sort()

        tag_poses = []
        result = FindTagResult(PoseStamped(), False)

        start_time = rospy.Time.now()
        current_time = rospy.Time.now()
        while current_time - start_time < self.timeout:
            current_time = rospy.Time.now()
            tag_poses = []  # reset buffer since num_samples not met
            for index in range(0, -len(self.tag_ring), -1):
                # iterate through detections in the order they were added
                index = (index + self.ring_index) % len(self.tag_ring)
                detection = self.tag_ring[index]
                if detection is None:
                    continue
                if detection.pose.header.stamp - start_time > self.stale_tag_time:
                    continue
                detect_id: List[int] = list(detection.id)
                detect_id.sort()
                if len(tag_id) != 0 and detect_id != tag_id:
                    continue

                pose = self.get_tag_pose_in(reference_frame, detection)
                if pose is None:
                    continue
                rospy.loginfo(f"Found a tag: {pose}. {len(tag_poses)} found")

                feedback = FindTagFeedback()
                feedback.sample = pose
                feedback.num_samples = len(tag_poses)
                self.action_server.publish_feedback(feedback)

                tag_poses.append(pose)
            if len(tag_poses) >= self.num_samples:
                result.success = True
                break

        if result.success:
            pose2ds = [Pose2d.from_ros_pose(msg.pose) for msg in tag_poses]
            filtered_pose = Pose2d.median(pose2ds)
            result_pose = tag_poses[-1]
            result_pose.pose = filtered_pose.to_ros_pose()
            result.pose = result_pose
            rospy.loginfo(f"Final tag pose. 2D: {filtered_pose}. 3D: {result_pose}")

        if result.success:
            self.action_server.set_succeeded(result)
        else:
            self.action_server.set_aborted(result)
