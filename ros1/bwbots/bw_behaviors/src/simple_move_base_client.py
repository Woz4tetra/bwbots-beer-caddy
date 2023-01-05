import copy
import rospy
import actionlib
import numpy as np
import tf2_ros
import tf2_geometry_msgs
from typing import Callable, Optional

from geometry_msgs.msg import PoseStamped
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import (
    MoveBaseAction,
    MoveBaseGoal,
    MoveBaseFeedback,
    MoveBaseResult,
)
from nav_msgs.srv import GetPlan


class SimpleMoveBaseClient:
    def __init__(
        self, move_base_namespace: str, base_frame: str, global_frame: str
    ) -> None:
        self.move_base_namespace = move_base_namespace
        self.base_frame = base_frame
        self.global_frame = global_frame
        self.move_base = actionlib.SimpleActionClient(
            self.move_base_namespace, MoveBaseAction
        )
        self.is_move_base_done = True
        self.move_base_succeeded = True

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.make_plan_srv = rospy.ServiceProxy(
            self.move_base_namespace + "/make_plan", GetPlan
        )

    def connect(self):
        rospy.loginfo("Connecting to move_base...")
        self.move_base.wait_for_server()
        rospy.loginfo("move_base connected")

    def wait(self, is_preempt_requested: Callable[[], bool]) -> int:
        rate = rospy.Rate(10.0)
        status = GoalStatus.PENDING
        while not self.is_move_base_done:
            if rospy.is_shutdown():
                rospy.loginfo("Received abort. Cancelling waypoint goal")
                status = GoalStatus.ABORTED
                self.move_base.cancel_goal()
                break

            if is_preempt_requested():
                rospy.loginfo("Received preempt. Cancelling waypoint goal")
                status = GoalStatus.ABORTED
                self.move_base.cancel_goal()
                break
            status = GoalStatus.ACTIVE
            rate.sleep()
        status = GoalStatus.SUCCEEDED
        return status

    def did_succeed(self):
        return self.move_base_succeeded

    def get_state(self) -> int:
        return self.move_base.get_state()

    def transform_to_global_frame(
        self, pose_stamped: PoseStamped, timeout=rospy.Duration(1.0)
    ) -> Optional[PoseStamped]:

        try:
            transform = self.tf_buffer.lookup_transform(
                self.global_frame, pose_stamped.header.frame_id, rospy.Time(0), timeout
            )
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as e:
            return None
        return tf2_geometry_msgs.do_transform_pose(pose_stamped, transform)

    def send_goal(
        self,
        pose_stamped: PoseStamped,
        feedback_callback: Callable[[MoveBaseFeedback], None],
    ):
        mb_goal = MoveBaseGoal()
        mb_goal.target_pose.header.frame_id = pose_stamped.header.frame_id
        mb_goal.target_pose.pose = pose_stamped.pose

        if not self.is_done():
            rospy.loginfo("move_base already has a goal. Cancelling for new one.")
            self.cancel()

        self.is_move_base_done = False
        self.move_base_succeeded = False
        self.move_base.send_goal(
            mb_goal, feedback_cb=feedback_callback, done_cb=self.move_base_done
        )

    def cancel(self, wait_for_done=False):
        self.move_base.cancel_all_goals()
        if wait_for_done:
            while not self.is_move_base_done:
                if rospy.is_shutdown():
                    break
                rospy.sleep(0.05)

    def move_base_done(self, goal_status: GoalStatus, result: MoveBaseResult):
        rospy.loginfo("move_base finished: %s. %s" % (goal_status, result))
        self.is_move_base_done = True
        self.move_base_succeeded = goal_status == GoalStatus.SUCCEEDED

    def is_done(self):
        return self.is_move_base_done

    def get_robot_pose(self) -> Optional[PoseStamped]:
        base_pose = PoseStamped()
        base_pose.header.frame_id = self.base_frame
        base_pose.pose.orientation.w = 1.0

        return self.transform_to_global_frame(base_pose)

    def goal_offset_radius(
        self,
        goal_pose: PoseStamped,
        offset_radius: float,
        use_last_pose_as_last_orientation: bool = False,
    ) -> Optional[PoseStamped]:
        """
        Get a pose at a radial distance along move_base's computed path to the goal.
        If all poses are less than the distance threshold, use the starting pose
        """
        assert goal_pose.header.frame_id == self.global_frame
        assert offset_radius >= 0.0

        if offset_radius == 0.0:
            return copy.deepcopy(goal_pose)

        start_pose = self.get_robot_pose()
        if start_pose is None:
            rospy.logwarn(
                f"Failed to get path to goal. Couldn't transform from {self.base_frame} to {self.global_frame}."
            )
            return None

        resp = self.make_plan_srv(
            start=start_pose, goal=goal_pose, tolerance=offset_radius
        )

        if resp and len(resp.plan.poses) > 0:
            result_pose = PoseStamped()
            last_pose = resp.plan.poses[-1]
            path_index = len(resp.plan.poses) - 1
            while (
                self.get_pose_distance(goal_pose, resp.plan.poses[path_index])
                < offset_radius
            ):
                path_index -= 1
                if path_index < 0:
                    rospy.logwarn(
                        "All poses in the plan are less than the offset radius '%s'. "
                        "Using starting pose." % offset_radius
                    )
                    path_index = 0
                    break
            result_pose = resp.plan.poses[path_index]
            if use_last_pose_as_last_orientation:
                result_pose.pose.orientation = last_pose.pose.orientation
            return result_pose
        else:
            rospy.logwarn(
                "Failed to get path to goal. move_base failed to produce a plan"
            )
            return None

    def get_pose_distance(self, pose_stamped1, pose_stamped2):
        """
        Compute the distance between two PoseStamped objects.
        Assuming both objects are in the same frame
        """
        assert (
            pose_stamped1.header.frame_id == pose_stamped2.header.frame_id
        ), "%s != %s" % (pose_stamped1.header.frame_id, pose_stamped2.header.frame_id)
        point1 = np.array(
            [
                pose_stamped1.pose.position.x,
                pose_stamped1.pose.position.y,
                pose_stamped1.pose.position.z,
            ],
            dtype=np.float64,
        )
        point2 = np.array(
            [
                pose_stamped2.pose.position.x,
                pose_stamped2.pose.position.y,
                pose_stamped2.pose.position.z,
            ],
            dtype=np.float64,
        )
        distance = np.linalg.norm(point2 - point1)
        return distance
