import rospy
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseFeedback

from bw_tools.robot_state import Pose2d
from bw_tools.simple_move_base_client import SimpleMoveBaseClient


class MoveBaseManager:
    def __init__(self, robot_frame: str, global_frame: str) -> None:
        self.client = SimpleMoveBaseClient("/move_base", robot_frame, global_frame)

    def send_goal_2d(self, goal_2d: Pose2d) -> None:
        pose = goal_2d.to_ros_pose()
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = self.client.global_frame
        pose_stamped.pose = pose
        self.send_goal(pose_stamped)

    def send_goal(self, goal: PoseStamped) -> None:
        self.client.connect()
        self.client.send_goal(goal, self.feedback_callback)

    def is_done(self) -> bool:
        return self.client.is_done()

    def did_succeed(self) -> bool:
        return self.client.did_succeed()

    def cancel(self) -> None:
        self.client.cancel()

    def feedback_callback(self, mb_feedback: MoveBaseFeedback) -> None:
        rospy.loginfo_throttle(0.5, f"Robot pose: {mb_feedback.base_position}")
