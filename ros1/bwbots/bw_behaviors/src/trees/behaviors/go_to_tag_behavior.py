from typing import Callable, Optional
import rospy
import py_trees
import py_trees_ros

from bw_interfaces.msg import GoToPoseAction, GoToPoseGoal

from trees.managers.tag_manager import TagManager



class GoToTagBehavior(py_trees_ros.actions.ActionClient):
    def __init__(self,
            x_offset: float,
            y_offset: float,
            theta_offset: float,
            tag_name_supplier: Callable[[], str],
            tag_manager: TagManager,
            valid_tag_window: Optional[rospy.Duration] = None,
            **kwargs):
        self.x_offset = x_offset
        self.y_offset = y_offset
        self.theta_offset = theta_offset
        self.valid_tag_window = valid_tag_window
        self.tag_name_supplier = tag_name_supplier
        self.tag_manager = tag_manager
        
        super().__init__("Go to tag",
            GoToPoseAction,
            action_namespace="/bw/go_to_pose")

        self.action_goal = GoToPoseGoal()
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
        self.action_goal.linear_zero_vel = kwargs.get("linear_zero_vel", 0.0001)
        self.action_goal.theta_max_vel = kwargs.get("theta_max_vel", 3.0)
        self.action_goal.theta_max_accel = kwargs.get("theta_max_accel", 1.0)
        self.action_goal.theta_min_vel = kwargs.get("theta_min_vel", 0.015)
        self.action_goal.theta_zero_vel = kwargs.get("theta_zero_vel", 0.0001)

    def update(self):
        if not self.sent_goal:
            tag_name = self.tag_name_supplier()
            if type(tag_name) != str:
                rospy.logwarn(f"Supplied tag name is not a string: {tag_name}")
                return py_trees.Status.FAILURE
            tag_pose_stamped = self.tag_manager.get_offset_tag(
                tag_name, self.x_offset, self.y_offset, self.theta_offset, self.valid_tag_window
            )
            if tag_pose_stamped is None:
                rospy.logwarn(f"Failed to get pose of tag {tag_name}")
                return py_trees.Status.FAILURE
            self.action_goal.goal.pose = tag_pose_stamped.pose
            self.action_goal.goal.header.frame_id = tag_pose_stamped.header.frame_id
        return super().update()
