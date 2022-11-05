import rospy
import tf2_ros
import actionlib

from geometry_msgs.msg import PoseStamped

from bw_interfaces.msg import FindTagAction, FindTagGoal, FindTagFeedback, FindTagResult

from bw_tools.robot_state import Pose2d


class FindTagCommand:
    def __init__(self) -> None:
        self.num_samples = rospy.get_param("~find_tag/num_samples", 10)
        self.sample_interval = rospy.get_param("~find_tag/sample_interval", 0.1)
        self.max_attempts = rospy.get_param("~find_tag/max_attempts", 10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.action_server = actionlib.ActionServer(
            "find_tag",
            FindTagAction,
            self.action_callback, 
            auto_start=False
        )
        self.action_server.start()
        rospy.loginfo("find_tag is ready")

    def action_callback(self, goal: FindTagGoal):
        tag_frame = goal.tag_frame_id
        reference_frame = goal.reference_frame_id
        tag_poses = []
        result = FindTagResult(PoseStamped(), True)

        sample_num = 0
        attempts = 0
        while sample_num < self.num_samples:
            tag_pose = self.lookup_tag(reference_frame, tag_frame)
            rospy.sleep(self.sample_interval)
            if tag_pose is None:
                attempts += 1
                if attempts >= self.max_attempts:
                    result = FindTagResult(PoseStamped(), False)
            else:
                attempts = 0
                tag_poses.append(tag_pose)

                feedback = FindTagFeedback()
                feedback.sample = tag_pose
                self.follow_waypoints_server.publish_feedback(feedback)

        if result.success:
            pose2ds = [Pose2d.from_ros_pose(ps.pose) for ps in tag_poses]
            filtered_pose = Pose2d.median(pose2ds)
            result_pose = tag_poses[-1]
            result_pose.pose = filtered_pose.to_ros_pose()
            result.pose = result_pose

        self.action_server.publish_result(result)
        if result.success:
            self.action_server.set_succeeded()
        else:
            self.action_server.set_aborted()
    
    def lookup_tag(self, reference_frame, tag_frame):
        try:
            current_tf = self.tf_buffer.lookup_transform(reference_frame, tag_frame, rospy.Time(0), rospy.Duration(1.0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            return None
        
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = reference_frame
        pose.pose.position = current_tf.transform.translation
        pose.pose.orientation = current_tf.transform.rotation
        return pose
