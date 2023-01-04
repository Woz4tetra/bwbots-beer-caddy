import math
import copy
import rospy
import tf2_ros
import actionlib
from typing import Optional

from actionlib_msgs.msg import GoalStatus
from vision_msgs.msg import Detection3DArray
from geometry_msgs.msg import PoseStamped, Pose

from simple_move_base_client import SimpleMoveBaseClient

from bw_interfaces.msg import (
    FollowDetectionAction,
    FollowDetectionGoal,
    FollowDetectionFeedback,
    FollowDetectionResult,
)


class FollowDetectionCommand:
    def __init__(self, move_base_client: SimpleMoveBaseClient) -> None:
        self.class_names_path = rospy.get_param("~class_names_path", "")
        self.stale_detections_timeout = rospy.Duration(
            rospy.get_param("~follow_detection/stale_detections_timeout", 1.0)
        )
        # if the detection used to plan moves this distance, replan
        self.replan_distance = rospy.get_param("~follow_detection/replan_distance", 0.5)

        # once the planner is within this distance of the goal, new detections will be ignored
        self.ignore_detections_once_in_distance = rospy.get_param(
            "~follow_detection/ignore_detections_once_in_distance", 0.75
        )

        self.move_base_client = move_base_client

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.detections_sub = rospy.Subscriber(
            "/zed/obj_det/detections",
            Detection3DArray,
            self.detections_callback,
            queue_size=10,
        )

        self.class_names = self.read_class_names(self.class_names_path)

        self.chase_pose = PoseStamped()
        self.chase_label = ""
        self.chase_distance = 0.0

        self.action_server = actionlib.SimpleActionServer(
            "follow_detection",
            FollowDetectionAction,
            execute_cb=self.action_callback,
            auto_start=False,
        )
        self.action_server.start()
        rospy.loginfo("follow_detection is ready")

    def detections_callback(self, msg: Detection3DArray) -> None:
        if len(self.chase_label) == 0:
            return
        closest_distance: Optional[float] = None
        closest_pose_stamped = PoseStamped()
        for detection in msg.detections:
            obj_hyp = detection.results[0]
            if self.get_label(obj_hyp.id) != self.chase_label:
                continue
            distance = self.get_pose_distance(obj_hyp.pose.pose)
            if closest_distance is None or distance < closest_distance:
                closest_distance = distance
                closest_pose_stamped.header = detection.header
                closest_pose_stamped.pose = obj_hyp.pose.pose
        if closest_distance is not None:
            chase_pose = self.move_base_client.transform_to_global_frame(
                closest_pose_stamped
            )
            if chase_pose is not None:
                self.chase_pose = chase_pose
                self.chase_distance = closest_distance

    def get_pose_distance(self, pose: Pose) -> float:
        return math.sqrt(
            pose.position.x * pose.position.x + pose.position.y * pose.position.y
        )

    def get_distance_between_poses(self, pose1: Pose, pose2: Pose) -> float:
        x = pose2.position.x - pose1.position.x
        y = pose2.position.y - pose1.position.y
        return math.sqrt(x * x + y * y)

    def read_class_names(self, path: str):
        if path and type(path) == str:
            rospy.loginfo(f"Loading class names from {path}")
            with open(path) as file:
                return file.read().splitlines()
        else:
            rospy.loginfo(f"No class names path defined. Not loading class names")
            return []

    def get_label(self, obj_id: int):
        index = obj_id & 0xFFFF
        if index < len(self.class_names):
            return self.class_names[index]
        else:
            return f"?? ({index})"

    def send_chase_goal(self, chase_pose: PoseStamped, offset_radius: float) -> bool:
        mb_goal = self.move_base_client.goal_offset_radius(chase_pose, offset_radius)
        if mb_goal is not None:
            rospy.loginfo(
                f"Chasing new detection named {self.chase_label}. Sending goal to move_base"
            )
            self.move_base_client.send_goal(mb_goal, self.move_base_feedback)
            return True
        else:
            return False

    def did_detection_move_too_much(self, sent_chase_pose: PoseStamped) -> bool:
        return (
            self.get_distance_between_poses(self.chase_pose.pose, sent_chase_pose.pose)
            > self.replan_distance
        )

    def is_detection_nearby(self, sent_chase_pose: PoseStamped):
        robot_pose = self.move_base_client.get_robot_pose()
        return (
            robot_pose is not None
            and self.get_distance_between_poses(robot_pose.pose, sent_chase_pose.pose)
            < self.ignore_detections_once_in_distance
        )

    def action_callback(self, goal: FollowDetectionGoal) -> None:
        self.chase_label = goal.class_name
        offset_radius = goal.offset_distance
        detection_timeout = goal.detection_timeout

        result = FollowDetectionResult()

        current_time = rospy.Time.now()
        aborted = False
        is_goal_active = False
        mb_result = -1
        sent_chase_pose = PoseStamped()
        rate = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            rate.sleep()
            current_time = rospy.Time.now()
            detection_duration: rospy.Duration = current_time - self.chase_pose.header.stamp
            if detection_duration > detection_timeout:
                rospy.logwarn(
                    f"Received no detections for {detection_timeout:0.3f} seconds."
                )
                aborted = True
                break
            if self.action_server.is_preempt_requested():
                aborted = True
                break
            if detection_duration > self.stale_detections_timeout:
                continue
            if not is_goal_active:
                sent_chase_pose = copy.deepcopy(self.chase_pose)
                is_goal_active = self.send_chase_goal(self.chase_pose, offset_radius)
            else:
                if self.did_detection_move_too_much(sent_chase_pose):
                    rospy.loginfo(
                        f"Chase goal {self.chase_label} moved by at least {self.replan_distance:0.4f}m. Replanning."
                    )
                    is_goal_active = False

                if self.is_detection_nearby(sent_chase_pose):
                    rospy.loginfo(
                        f"Robot is within {self.ignore_detections_once_in_distance:0.4f}m of detection. Waiting for move_base to finish."
                    )
                    mb_result = self.move_base_client.wait(
                        lambda: self.action_server.is_preempt_requested()
                    )
                    break

                if self.move_base_client.is_done():
                    mb_result = self.move_base_client.get_state()
                    break

        if aborted:
            self.action_server.set_aborted(
                result, f"Interrupted while following detection {self.chase_label}"
            )
        else:
            if mb_result != GoalStatus.SUCCEEDED:
                rospy.loginfo(f"Move base is reporting a failure code: {mb_result}")
                result.success = False
            else:
                result.success = True

            if result.success:
                rospy.loginfo(f"Reached detection {self.chase_label}")
            else:
                rospy.loginfo(f"Failed to reach detection {self.chase_label}")
            self.action_server.set_succeeded(result)

    def move_base_feedback(self, mb_feedback):
        feedback = FollowDetectionFeedback()
        feedback.current_pose = mb_feedback.base_position
        feedback.chase_pose = self.chase_pose
        feedback.distance_from_detection = self.chase_distance
        self.action_server.publish_feedback(feedback)
