from py_trees.behaviour import Behaviour
from py_trees.composites import Sequence

from bw_behaviors.behaviors.follow_waypoint import FollowWaypoint
from bw_behaviors.behaviors.simple_go_to_waypoint import SimpleGoToWaypoint
from bw_behaviors.container import Container
from bw_tools.structs.go_to_goal import GoToPoseGoal


def get_dock_stage_1(container: Container) -> GoToPoseGoal:
    offset = container.named_offsets_manager.get("dock_stage_1")
    undock_goal = GoToPoseGoal.from_xyt(
        "",  # pose is relative to waypoint which doesn't have a frame id
        offset.x,
        offset.y,
        offset.theta,
    )

    undock_goal.linear_min_vel = 0.15
    undock_goal.theta_min_vel = 0.1
    undock_goal.xy_tolerance = 0.025
    undock_goal.yaw_tolerance = 0.05
    undock_goal.timeout = 30.0
    undock_goal.reference_linear_speed = 0.5
    undock_goal.linear_max_accel = 0.25
    undock_goal.rotate_in_place_start = True
    undock_goal.rotate_while_driving = False
    undock_goal.reference_angular_speed = 2.0
    undock_goal.allow_reverse = False

    return undock_goal


def get_dock_stage_2(container: Container) -> GoToPoseGoal:
    offset = container.named_offsets_manager.get("dock_stage_2")
    undock_goal = GoToPoseGoal.from_xyt(
        "",  # pose is relative to waypoint which doesn't have a frame id
        offset.x,
        offset.y,
        offset.theta,
    )
    undock_goal.xy_tolerance = 0.05
    undock_goal.yaw_tolerance = 0.5
    undock_goal.linear_min_vel = 0.2
    undock_goal.theta_min_vel = 0.0
    undock_goal.reference_linear_speed = 10.0
    undock_goal.reference_angular_speed = 3.0
    undock_goal.linear_max_accel = 0.5
    undock_goal.allow_reverse = False
    undock_goal.rotate_in_place_start = True
    undock_goal.rotate_while_driving = False
    undock_goal.timeout = 60.0

    return undock_goal


def make_dock(container: Container) -> Behaviour:
    dock_prep_offset = container.named_offsets_manager.get("dock_prep")
    return Sequence(
        "dock_actions",
        memory=True,
        children=[
            FollowWaypoint(container, container.dock_waypoint_name, dock_prep_offset),
            SimpleGoToWaypoint(container, container.dock_waypoint_name, get_dock_stage_1(container)),
            SimpleGoToWaypoint(container, container.dock_waypoint_name, get_dock_stage_2(container)),
        ],
    )
