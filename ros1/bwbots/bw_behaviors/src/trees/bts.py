import rospy
import py_trees
import py_trees_ros
from bw_tools.robot_state import Pose2d

from bw_interfaces.msg import ShuffleUntilChargingAction, ShuffleUntilChargingGoal

from trees.behaviors.follow_waypoint_behavior import FollowWaypointBehavior
from trees.behaviors.find_tag_behavior import FindTagBehavior
from trees.behaviors.go_to_tag_behavior import GoToTagBehavior
from trees.behaviors.follow_tag_pose import FollowTagBehavior
from trees.behaviors.rotate_in_place_behavior import RotateInPlaceBehavior
from trees.behaviors.stop_driving_behavior import StopDrivingBehavior
from trees.behaviors.go_to_pose_behavior import GoToPoseBehavior
from trees.behaviors.is_dock_tag_near_behavior import IsDockTagNearBehavior
from trees.behaviors.save_tag_as_waypoint_behavior import SaveTagAsWaypoint

from trees.decorators.stop_driving_decorator import StopDrivingDecorator
from trees.decorators.repeat_n_times_decorator import RepeatNTimesDecorator


class BehaviorTrees:
    def __init__(self) -> None:
        self.dock_tag_id = [11]
        self.dock_tag_name = "dock_tag"
        self.global_frame = "map"
        self.dock_reference_frame = "odom"
        self.robot_frame = "base_link"
        self.dock_prep_waypoint = "dock_prep"
        self.bt_cache = {}

    def check_cache(self, name, init_fn):
        if name in self.bt_cache:
            return self.bt_cache[name]
        else:
            self.bt_cache[name] = init_fn()
            return self.bt_cache[name]

    def drive_to_dock_prep(self):
        return self.check_cache("drive_to_dock_prep", lambda: FollowWaypointBehavior(self.dock_prep_waypoint))
    
    def go_to_tag_stage1(self):
        return self.check_cache("go_to_tag_stage1", lambda: GoToTagBehavior(
            -0.5,
            0.05,
            self.dock_tag_name,
            controller_type="strafe1",
            linear_min_vel=0.1,
            xy_tolerance=0.025,
            yaw_tolerance=0.025,
            timeout=10.0,
            reference_linear_speed=5.0,
            rotate_in_place_start=True,
            rotate_while_driving=False,
            rotate_in_place_end=True,
            reference_angular_speed=2.0,
            allow_reverse=True
        ))
        
    def go_to_tag_stage2(self):
        return self.check_cache("go_to_tag_stage2", lambda: GoToTagBehavior(
            -0.05,
            0.05,
            self.dock_tag_name,
            controller_type="strafe2",
            xy_tolerance=0.05,
            yaw_tolerance=0.1,
            linear_min_vel=0.4,
            reference_linear_speed=10.0,
            reference_angular_speed=3.0,
            linear_max_accel=5.0,
            allow_reverse=False,
            rotate_in_place_start=False,
            rotate_while_driving=True,
            rotate_in_place_end=False
        ))
    
    def find_tag(self):
        return self.check_cache("find_tag", lambda: FindTagBehavior(self.dock_tag_id, self.dock_reference_frame, self.dock_tag_name))
        
    def shuffle_until_charging(self):
        shuffle_goal = ShuffleUntilChargingGoal()
        shuffle_goal.timeout = rospy.Duration(20.0)
        return self.check_cache("timeout", lambda: py_trees_ros.actions.ActionClient(
            "Shuffle until charging",
            ShuffleUntilChargingAction,
            shuffle_goal,
            "/bw/shuffle_until_charging"
        ))

    def follow_tag(self):
        return self.check_cache("follow_tag", lambda: FollowTagBehavior(0.0, -0.7, self.dock_tag_name))

    def rotate_in_place(self):
        return self.check_cache("rotate_in_place", lambda: RotateInPlaceBehavior(0.25, 10.0))

    def search_for_tag(self):
        return self.check_cache("search_for_tag", lambda: py_trees.composites.Parallel(
            "Search for tag",
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE,
            children=[self.rotate_in_place(), StopDrivingDecorator(self.find_tag())]
        ))
    
    def stop_driving(self):
        return self.check_cache("stop_driving", lambda: StopDrivingBehavior(pause=0.25))
        
    def drive_backwards(self):
        return self.check_cache("drive_backwards", lambda: GoToPoseBehavior(
            Pose2d(-0.7), self.robot_frame,
            controller_type="strafe2",
            xy_tolerance=0.15,
            yaw_tolerance=0.15,
            linear_min_vel=0.4,
            reference_linear_speed=1.0,
            reference_angular_speed=3.0,
            linear_max_accel=5.0,
            allow_reverse=True,
            rotate_in_place_start=False,
            rotate_while_driving=False,
            rotate_in_place_end=False
        ))
    
    def is_dock_tag_near(self):
        return self.check_cache("is_dock_tag_near", lambda: IsDockTagNearBehavior(self.dock_tag_id, self.robot_frame, 1.0))
    
    def save_tag_as_waypoint(self):
        return self.check_cache("save_tag_as_waypoint", lambda: SaveTagAsWaypoint(-0.7, 0.0, self.dock_prep_waypoint, self.global_frame))

    def dock(self):
        return py_trees.composites.Sequence("Dock", [
            self.drive_to_dock_prep(),
            RepeatNTimesDecorator(py_trees.composites.Selector("Search tag stage 0", [
                self.find_tag(), self.search_for_tag(), self.rotate_in_place(), self.follow_tag()
            ]), attempts=2),
            RepeatNTimesDecorator(py_trees.composites.Selector("Search tag stage 1", [
                py_trees.composites.Sequence("Go to tag stage 1", [
                    self.go_to_tag_stage1(), self.find_tag()
                ]),
                self.search_for_tag()
            ]), attempts=2),
            self.go_to_tag_stage2(),
            self.shuffle_until_charging()
        ])
    
    def undock(self):
        return py_trees.composites.Sequence("Undock", [
            self.drive_backwards(),
            py_trees.composites.Selector("Verify tag precense", [
                RepeatNTimesDecorator(py_trees.composites.Selector("Search for tag", [
                    self.find_tag(), self.search_for_tag()
                ])),
                self.is_dock_tag_near(),
                self.follow_tag()
            ]),
            self.save_tag_as_waypoint()
        ])
