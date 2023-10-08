from bw_behaviors.managers.charge_manager import ChargeManager
from bw_behaviors.managers.mode_manager import ModeManager
from bw_behaviors.managers.motors_enabled_manager import MotorsEnabledManager
from bw_behaviors.managers.move_base_manager import MoveBaseManager
from bw_behaviors.managers.named_offsets_manager import NamedOffsetsManager
from bw_behaviors.managers.reset_localization_manager import ResetLocalizationManager
from bw_behaviors.managers.simple_go_to_manager import SimpleGoToManager
from bw_behaviors.managers.teach_waypoint_manager import TeachWaypointManager
from bw_behaviors.managers.waypoint_manager import WaypointManager
from bw_tools.typing.basic import get_param


class Container:
    def __init__(self) -> None:
        self.robot_frame = get_param("~robot_frame", "base_link")
        self.global_frame = get_param("~global_frame", "map")
        named_offsets = get_param("~offsets", None)
        self.dock_waypoint_name = get_param("~dock_waypoint", "dock")
        self.charging_voltage_threshold = get_param("~charging_voltage_threshold", 12.2)
        self.charging_current_threshold = get_param("~charging_current_threshold", 0.25)

        self.mode_manager = ModeManager()
        self.simple_go_to_pose = SimpleGoToManager()
        self.motors_enabled_manager = MotorsEnabledManager()
        self.waypoint_manager = WaypointManager()
        self.reset_localization_manager = ResetLocalizationManager()
        self.charge_manager = ChargeManager()
        self.teach_waypoint_manager = TeachWaypointManager()
        self.move_base_manager = MoveBaseManager(self.robot_frame, self.global_frame)
        self.named_offsets_manager = NamedOffsetsManager(named_offsets)
