from bw_behaviors.managers.mode_manager import ModeManager
from bw_behaviors.managers.motors_enabled_manager import MotorsEnabledManager
from bw_behaviors.managers.reset_localization_manager import ResetLocalizationManager
from bw_behaviors.managers.simple_go_to_manager import SimpleGoToManager
from bw_behaviors.managers.waypoint_manager import WaypointManager
from bw_tools.typing.basic import get_param


class Container:
    def __init__(self) -> None:
        self.robot_frame = get_param("~robot_frame", "base_link")
        self.dock_prep_offset = get_param("~dock_prep_offset", -0.7)
        self.dock_waypoint_name = get_param("~dock_waypoint", "dock")

        self.mode_manager = ModeManager()
        self.simple_go_to_pose = SimpleGoToManager()
        self.motors_enabled_manager = MotorsEnabledManager()
        self.waypoint_manager = WaypointManager()
        self.reset_localization_manager = ResetLocalizationManager()
