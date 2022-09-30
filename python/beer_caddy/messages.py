from dataclasses import dataclass
from typing import List


@dataclass
class Odometry:
    timestamp = 0.0
    x = 0.0
    y = 0.0
    theta = 0.0
    vx = 0.0
    vy = 0.0
    vt = 0.0


@dataclass
class ModuleState:
    timestamp = 0.0
    azimuth = 0.0
    wheel_position = 0.0
    wheel_velocity = 0.0


class BwModuleStates:
    def __init__(self, states=None) -> None:
        if states is None:
            states = []
        self.states: List[ModuleState] = states
