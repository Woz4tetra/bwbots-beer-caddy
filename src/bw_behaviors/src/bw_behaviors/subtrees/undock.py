from py_trees.behaviour import Behaviour
from py_trees.composites import Sequence

from bw_behaviors.behaviors.is_charging import IsCharging
from bw_behaviors.behaviors.set_motors_enabled import SetMotorsEnabled
from bw_behaviors.behaviors.simple_go_to_pose import SimpleGoToPose
from bw_behaviors.behaviors.teach_waypoint import TeachWaypoint
from bw_behaviors.behaviors.teleport_to_waypoint import TeleportToWaypoint
from bw_behaviors.container import Container
from bw_tools.structs.go_to_goal import GoToPoseGoal


def make_undock(container: Container) -> Behaviour:
    dock_prep_offset = container.named_offsets_manager.get("dock_prep")
    undock_goal = GoToPoseGoal.from_xyt(
        container.robot_frame,
        dock_prep_offset.x,
        dock_prep_offset.y,
        dock_prep_offset.theta,
    )
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
            IsCharging(container, container.charging_voltage_threshold, container.charging_current_threshold),
            TeleportToWaypoint(container, container.dock_waypoint_name),
            SetMotorsEnabled(container, True),
            SimpleGoToPose(container, undock_goal),
            TeachWaypoint(container, container.dock_waypoint_name),
        ],
    )