from typing import Dict, List
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
from trees.behaviors.is_tag_near_behavior import IsTagNearBehavior
from trees.behaviors.save_tag_as_waypoint_behavior import SaveTagAsWaypointBehavior
from trees.behaviors.set_robot_state_behavior import SetRobotStateBehavior
from trees.behaviors.is_charging_behavior import IsChargingBehavior
from trees.behaviors.set_pose_to_waypoint_behavior import SetPoseToWaypointBehavior
from trees.behaviors.has_drink_behavior import HasDrinkBehavior

from trees.decorators.stop_driving_decorator import StopDrivingDecorator
from trees.decorators.repeat_n_times_decorator import RepeatNTimesDecorator
from trees.decorators.pause_before_running import PauseBeforeRunning
from trees.decorators.repeat_until_no_drink_missions_decorator import RepeatUntilNoDrinkMissionsDecorator
from trees.decorators.select_by_supplier_decorator import SelectBySupplierDecorator

from trees.managers.tag_manager import TagManager
from trees.managers.waypoint_manager import WaypointManager
from trees.managers.drink_mission_manager import DrinkMissionManager


class BehaviorTrees:
    def __init__(self) -> None:
        tag_mapping: Dict[str: Dict] = rospy.get_param("~tag_mapping", {})
        
        self.dock_tag_name: str = rospy.get_param("~dock_tag_name", "dock_tag")
        
        self.global_frame: str = rospy.get_param("~global_frame", "map")
        self.go_to_tag_reference_frame: str = rospy.get_param("~go_to_tag_reference_frame", "odom")
        self.robot_frame: str = rospy.get_param("~robot_frame", "base_link")
        
        self.has_drink_mass_threshold: float = rospy.get_param("~has_drink_mass_threshold", 0.1)
        self.dock_prep_offset: float = rospy.get_param("~dock_prep_offset", -0.7)
        self.dispenser_prep_offset: float = rospy.get_param("~dispenser_prep_offset", -0.7)
        self.valid_tag_window: float = rospy.get_param("~valid_tag_window", 1.5)

        self.bt_cache = {}

        self.tag_manager = TagManager(valid_tag_window=self.valid_tag_window)
        assert self.dock_tag_name in tag_mapping, f"tag_mapping: {tag_mapping}"
        for tag_name, values in tag_mapping.items():
            self.tag_manager.register_tag(
                name=tag_name,
                tag_id=values["id"],
                prep=values["prep"],
                tag_type=values["type"],
                reference_frame=self.go_to_tag_reference_frame
            )

        self.dock_tag_supplier = lambda: self.dock_tag_name
        self.dock_prep_supplier = lambda: self.tag_manager.get_tag(self.dock_tag_name).prep
        
        self.waypoint_manager = WaypointManager()
        self.drink_mission_manager = DrinkMissionManager()

    def check_cache(self, name, init_fn):
        if name in self.bt_cache:
            return self.bt_cache[name]
        else:
            self.bt_cache[name] = init_fn()
            return self.bt_cache[name]

    def follow_waypoint(self, waypoint_name_supplier):
        return self.check_cache("follow_waypoint", lambda: FollowWaypointBehavior(waypoint_name_supplier, self.waypoint_manager))
    
    def go_to_dock_stage1(self):
        return self.check_cache("go_to_dock_stage1", lambda: GoToTagBehavior(
            -0.5,
            0.05,
            self.dock_tag_supplier,
            self.tag_manager,
            frame_id=self.go_to_tag_reference_frame,
            controller_type="strafe1",
            linear_min_vel=0.1,
            theta_min_vel=0.015,
            xy_tolerance=0.025,
            yaw_tolerance=0.025,
            timeout=10.0,
            reference_linear_speed=0.5,
            rotate_in_place_start=True,
            rotate_while_driving=False,
            rotate_in_place_end=True,
            reference_angular_speed=2.0,
            allow_reverse=False,
            failure_on_pose_failure=False
        ))
        
    def go_to_dock_stage2(self):
        return self.check_cache("go_to_dock_stage2", lambda: GoToTagBehavior(
            -0.05,
            0.05,
            self.dock_tag_supplier,
            self.tag_manager,
            frame_id=self.go_to_tag_reference_frame,
            controller_type="strafe2",
            xy_tolerance=0.05,
            yaw_tolerance=0.25,
            linear_min_vel=0.4,
            theta_min_vel=0.02,
            reference_linear_speed=10.0,
            reference_angular_speed=3.0,
            linear_max_accel=5.0,
            allow_reverse=False,
            rotate_in_place_start=False,
            rotate_while_driving=True,
            rotate_in_place_end=False,
            failure_on_pose_failure=True
        ))
    
    def find_tag(self, tag_name_supplier):
        return self.check_cache("find_tag", lambda: FindTagBehavior(tag_name_supplier, self.tag_manager, 10.0))
    
    def shuffle_until_charging(self):
        shuffle_goal = ShuffleUntilChargingGoal()
        shuffle_goal.timeout = rospy.Duration(20.0)
        return self.check_cache("shuffle_until_charging", lambda: py_trees_ros.actions.ActionClient(
            "Shuffle until charging",
            ShuffleUntilChargingAction,
            shuffle_goal,
            "/bw/shuffle_until_charging"
        ))

    def follow_tag(self, tag_name_supplier, x_offset, y_offset):
        return FollowTagBehavior(x_offset, y_offset, tag_name_supplier, self.tag_manager)

    def rotate_in_place(self):
        return self.check_cache("rotate_in_place", lambda: RotateInPlaceBehavior(1.5, 10.0))

    def search_for_tag(self, tag_name_supplier):
        return py_trees.composites.Parallel(
            "Search for tag",
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE,
            children=[self.rotate_in_place(), StopDrivingDecorator(self.find_tag(tag_name_supplier))]
        )
    
    def stop_driving(self):
        return self.check_cache("stop_driving", lambda: StopDrivingBehavior(pause=0.25))

    def drive_off_dock(self):
        return self.check_cache("drive_off_dock", lambda: GoToPoseBehavior(
            Pose2d(self.dock_prep_offset, 0.0, 0.0),
            self.robot_frame,
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
    
    def is_tag_near(self, tag_name_supplier):
        return self.check_cache("is_tag_near", lambda: IsTagNearBehavior(self.robot_frame, tag_name_supplier, self.tag_manager, 1.0))
    
    def save_tag_as_waypoint(self, x_offset, y_offset, tag_name_supplier, waypoint_name_supplier):
        return SaveTagAsWaypointBehavior(x_offset, y_offset, self.global_frame, waypoint_name_supplier, tag_name_supplier, self.tag_manager)

    def enable_motors(self):
        return self.check_cache("enable_motors", lambda: SetRobotStateBehavior(True))

    def disable_motors(self):
        return self.check_cache("disable_motors", lambda: SetRobotStateBehavior(False))

    def is_charging(self):
        return self.check_cache("is_charging", lambda: IsChargingBehavior())

    def set_robot_to_waypoint(self, waypoint_name_supplier):
        return SetPoseToWaypointBehavior(waypoint_name_supplier, self.waypoint_manager)

    def has_drink(self):
        return self.check_cache("has_drink", lambda: HasDrinkBehavior(1.0, self.has_drink_mass_threshold, False))

    def has_no_drink(self):
        return self.check_cache("has_no_drink", lambda: HasDrinkBehavior(1.0, self.has_drink_mass_threshold, True))

    def dispenser_prep_supplier(self):
        def supplier():
            mission = self.drink_mission_manager.get_active()
            if mission is None:
                return None
            else:
                return self.tag_manager.get_tag(self.drink_mission_manager.get_active().drink_dispenser_waypoint).prep
        return supplier()

    def dispenser_tag_supplier(self):
        def supplier():
            mission = self.drink_mission_manager.get_active()
            if mission is None:
                return None
            else:
                return self.drink_mission_manager.get_active().drink_dispenser_waypoint
        return supplier()

    def delivery_supplier(self):
        def supplier():
            mission = self.drink_mission_manager.get_active()
            if mission is None:
                return None
            else:
                return self.drink_mission_manager.get_active().delivery_waypoint
        return supplier()

    def dispenser_type_supplier(self):
        def supplier():
            mission = self.drink_mission_manager.get_active()
            if mission is None:
                return None
            else:
                return self.tag_manager.get_tag(self.drink_mission_manager.get_active().delivery_waypoint).tag_type
        return supplier()

    def go_to_dispenser_type_A0_stage1(self):
        return self.check_cache("go_to_dispenser_type_A0_stage1", lambda: GoToTagBehavior(
            -0.5,
            0.075,
            self.dispenser_tag_supplier,
            self.tag_manager,
            frame_id=self.go_to_tag_reference_frame,
            controller_type="strafe1",
            linear_min_vel=0.1,
            theta_min_vel=0.02,
            xy_tolerance=0.025,
            yaw_tolerance=0.0125,
            timeout=10.0,
            reference_linear_speed=0.5,
            rotate_in_place_start=True,
            rotate_while_driving=False,
            rotate_in_place_end=True,
            reference_angular_speed=2.0,
            allow_reverse=False,
            failure_on_pose_failure=False
        ))

    def go_to_dispenser_type_A0_stage2(self):
        return self.check_cache("go_to_dispenser_type_A0_stage2", lambda: GoToTagBehavior(
            -0.05,
            0.075,
            self.dispenser_tag_supplier,
            self.tag_manager,
            frame_id=self.go_to_tag_reference_frame,
            controller_type="strafe2",
            xy_tolerance=0.05,
            yaw_tolerance=0.25,
            linear_min_vel=0.4,
            theta_min_vel=0.02,
            reference_linear_speed=10.0,
            reference_angular_speed=3.0,
            linear_max_accel=5.0,
            allow_reverse=False,
            rotate_in_place_start=False,
            rotate_while_driving=True,
            rotate_in_place_end=False,
            failure_on_pose_failure=True
        ))

    def dock(self):
        return py_trees.composites.Sequence("Dock", [
            py_trees.decorators.Inverter(self.is_charging()),
            self.enable_motors(),
            self.follow_waypoint(self.dock_prep_supplier),
            RepeatNTimesDecorator(py_trees.composites.Selector("Search tag stage 0", [
                PauseBeforeRunning(self.find_tag(self.dock_tag_supplier), 3.0),
                self.search_for_tag(self.dock_tag_supplier),
                self.rotate_in_place(),
                self.follow_tag(self.dock_tag_supplier, 0.0, self.dock_prep_offset)
            ]), attempts=2),
            RepeatNTimesDecorator(py_trees.composites.Selector("Search tag stage 1", [
                py_trees.composites.Sequence("Go to tag stage 1", [
                    self.go_to_dock_stage1(), PauseBeforeRunning(self.find_tag(self.dock_tag_supplier), 3.0)
                ]),
                self.search_for_tag(self.dock_tag_supplier)
            ]), attempts=2),
            self.go_to_dock_stage2(),
            self.shuffle_until_charging(),
            self.disable_motors()
        ])
    
    def undock(self):
        return py_trees.composites.Sequence("Undock", [
            self.is_charging(),
            self.enable_motors(),
            self.set_robot_to_waypoint(self.dock_tag_supplier),
            self.drive_off_dock(),
            py_trees.decorators.Inverter(self.is_charging()),
            py_trees.composites.Selector("Verify tag precense", [
                RepeatNTimesDecorator(py_trees.composites.Selector("Search for tag", [
                    PauseBeforeRunning(self.find_tag(self.dock_tag_supplier), 3.0), self.search_for_tag(self.dock_tag_supplier)
                ]), attempts=2),
                self.is_tag_near(self.dock_tag_supplier),
            ]),
            self.save_tag_as_waypoint(0.0, 0.0, self.dock_tag_supplier, self.dock_tag_supplier),
            self.save_tag_as_waypoint(self.dock_prep_offset, 0.0, self.dock_tag_supplier, self.dock_prep_supplier)
        ])

    def dispenser_stage1_sequence(self):
        selected_stage1 = SelectBySupplierDecorator({
            "A0": self.go_to_dispenser_type_A0_stage1()
        }, self.dispenser_type_supplier)

        return py_trees.composites.Sequence("Go to dispenser stage 1", [
                RepeatNTimesDecorator(py_trees.composites.Selector("Go to dispenser stage 1 selector", [
                py_trees.composites.Sequence("Go to dispenser stage 1 sequence", [
                    selected_stage1,
                    PauseBeforeRunning(self.find_tag(self.dispenser_tag_supplier), 3.0)
                ]),
                self.search_for_tag(self.dispenser_tag_supplier)
            ]), attempts=2),
            self.save_tag_as_waypoint(0.0, 0.0, self.dispenser_tag_supplier, self.dispenser_tag_supplier),
            self.save_tag_as_waypoint(0.0, self.dispenser_prep_offset, self.dispenser_tag_supplier, self.dispenser_prep_supplier),
        ])

    def dispenser_stage2_sequence(self):
        return SelectBySupplierDecorator({
            "A0": self.go_to_dispenser_type_A0_stage2()
        }, self.dispenser_type_supplier)

    def collect_drink(self):
        return py_trees.composites.Sequence("Collect drink", [
            self.follow_waypoint(self.dispenser_prep_supplier),
            RepeatNTimesDecorator(py_trees.composites.Selector("Search dispenser stage 0", [
                PauseBeforeRunning(self.find_tag(self.dispenser_tag_supplier), 3.0),
                self.search_for_tag(self.dispenser_tag_supplier),
                self.rotate_in_place(),
                self.follow_tag(self.dispenser_tag_supplier, 0.0, 0.0)  # TODO find actual offsets. Add angle
            ]), attempts=2),
            self.dispenser_stage1_sequence(),
            self.dispenser_stage2_sequence(),
        ])

    def deliver_drink(self):
        return py_trees.composites.Sequence("Deliver drink", [
            self.follow_waypoint(self.delivery_supplier),
            # self.follow_person(),
        ])

    def drink_mission(self):
        return py_trees.composites.Sequence("Drink mission", [
            self.has_no_drink(),
            py_trees.decorators.FailureIsSuccess(self.undock()),
            RepeatUntilNoDrinkMissionsDecorator(
                py_trees.composites.Sequence("Run through mission queue", [
                    self.collect_drink(),
                    self.has_drink(),
                    self.deliver_drink(),
                ]),
                self.drink_mission_manager),
            self.dock()
        ])

