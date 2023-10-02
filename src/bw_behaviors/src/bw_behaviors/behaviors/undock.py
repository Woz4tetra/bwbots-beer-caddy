from py_trees.behaviour import Behaviour
from py_trees.composites import Sequence

from bw_behaviors.behaviors.set_motors_enabled import SetMotorsEnabled
from bw_behaviors.behaviors.simple_go_to_pose import SimpleGoToPose
from bw_behaviors.container import Container
from bw_tools.dataclasses.go_to_goal import GoToPoseGoal


def make_undock(container: Container) -> Behaviour:
    undock_goal = GoToPoseGoal.from_xyt(container.robot_frame, container.dock_prep_offset, 0.0, 0.0)
    undock_goal.xy_tolerance = 0.3
    undock_goal.yaw_tolerance = 0.3
    undock_goal.linear_min_vel = 0.4
    undock_goal.reference_linear_speed = 1.5
    undock_goal.reference_angular_speed = 3.0
    undock_goal.linear_max_accel = 5.0
    undock_goal.timeout = 10.0
    undock_goal.allow_reverse = True
    undock_goal.rotate_in_place_start = False
    undock_goal.rotate_while_driving = False

    return Sequence(
        "undock_actions",
        memory=True,
        children=[
            SetMotorsEnabled(container, True),
            SimpleGoToPose(container, undock_goal.to_msg()),
        ],
    )
