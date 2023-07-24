import math
from enum import Enum, auto
from dataclasses import dataclass
from bw_tools.robot_state import Pose2d


@dataclass
class TrapezoidalProfileConfig:
    min_speed: float
    max_speed: float
    acceleration: float


@dataclass
class ControllerStateMachineConfig:
    allow_reverse: bool
    rotate_while_driving: bool
    rotate_trapezoid: TrapezoidalProfileConfig
    drive_to_pose_trapezoid: TrapezoidalProfileConfig
    pose_tolerance: Pose2d
    settle_time: float = 1.0
    rotate_in_place_start: bool = True
    rotate_in_place_end: bool = True
    rotate_angle_threshold: float = math.pi / 4.0
    strafe_angle_threshold: float = math.pi / 4.0


class ControllerState(Enum):
    IDLE = auto()
    ROTATE_IN_PLACE_START = auto()
    DRIVE_TO_POSE = auto()
    ROTATE_IN_PLACE_END = auto()
