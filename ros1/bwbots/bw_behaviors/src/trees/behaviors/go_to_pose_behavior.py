import rospy
import py_trees
import py_trees_ros

from bw_interfaces.msg import GoToPoseAction, GoToPoseGoal

from bw_tools.robot_state import Pose2d


class GoToPoseBehavior(py_trees_ros.actions.ActionClient):
    def __init__(self, pose2d: Pose2d, frame_id="odom", **kwargs):
        self.blackboard = py_trees.blackboard.Blackboard()
        
        super().__init__("Go to pose",
            GoToPoseAction,
            action_namespace="/bw/go_to_pose")

        self.action_goal = GoToPoseGoal()
        self.action_goal.goal.header.frame_id = frame_id
        self.action_goal.goal.pose = pose2d.to_ros_pose()
        self.action_goal.controller_type = kwargs.get("controller_type", "strafe1")
        self.action_goal.xy_tolerance = kwargs.get("xy_tolerance", 0.05)
        self.action_goal.yaw_tolerance = kwargs.get("yaw_tolerance", 0.15)
        self.action_goal.timeout = rospy.Duration(kwargs.get("timeout", 2.0))
        self.action_goal.reference_linear_speed = kwargs.get("reference_linear_speed", 0.5)
        self.action_goal.reference_angular_speed = kwargs.get("reference_angular_speed", 3.0)
        self.action_goal.allow_reverse = kwargs.get("allow_reverse", True)
        self.action_goal.rotate_in_place_start = kwargs.get("rotate_in_place_start", True)
        self.action_goal.rotate_while_driving = kwargs.get("rotate_while_driving", True)
        self.action_goal.rotate_in_place_end = kwargs.get("rotate_in_place_end", True)
        self.action_goal.linear_max_vel = kwargs.get("linear_max_vel", 1.0)
        self.action_goal.linear_max_accel = kwargs.get("linear_max_accel", 2.0)
        self.action_goal.linear_min_vel = kwargs.get("linear_min_vel", 0.015)
        self.action_goal.linear_zero_vel = kwargs.get("linear_zero_vel", 0.014)
        self.action_goal.theta_max_vel = kwargs.get("theta_max_vel", 3.0)
        self.action_goal.theta_max_accel = kwargs.get("theta_max_accel", 1.0)
        self.action_goal.theta_min_vel = kwargs.get("theta_min_vel", 0.015)
        self.action_goal.theta_zero_vel = kwargs.get("theta_zero_vel", 0.0001)
