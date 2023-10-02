from bw_behaviors.managers.mode_manager import ModeManager
from bw_behaviors.managers.motors_enabled_manager import MotorsEnabledManager
from bw_behaviors.managers.simple_go_to_manager import SimpleGoToManager
from bw_tools.typing.basic import get_param


class Container:
    def __init__(self) -> None:
        self.robot_frame = get_param("~robot_frame", "base_link")
        self.dock_prep_offset = get_param("~dock_prep_offset", -0.7)

        self.mode_manager = ModeManager()
        self.simple_go_to_pose = SimpleGoToManager()
        self.motors_enabled_manager = MotorsEnabledManager()
