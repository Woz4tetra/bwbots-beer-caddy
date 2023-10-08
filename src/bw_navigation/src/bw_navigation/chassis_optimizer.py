#!/usr/bin/env python3
from typing import Any

import control
import numpy as np

from bw_navigation.base_optimizer import BaseOptimizer
from bw_navigation.optimization_helpers import (
    SystemConstraints,
    SystemInputU,
    SystemOutputY,
    SystemStateX,
    module_system_update,
)
from bw_tools.robot_state import Velocity
from bw_tools.structs.go_to_goal import GoToPoseGoal


class ChassisOptimizer(BaseOptimizer[Velocity]):
    def __init__(self) -> None:
        input_names = ("vx", "vy", "vt")
        state_names = ("x", "y", "t")

        system = control.NonlinearIOSystem(
            self.system_update,
            self.system_output,
            name="go_to_pose",
            states=state_names,
            inputs=input_names,
            outputs=state_names,
        )

        super().__init__(system, Velocity())

    def system_update(
        self, timestamp: float, state_x: SystemStateX, input_u: SystemInputU, params: Any
    ) -> SystemOutputY:
        return module_system_update(state_x, input_u)

    def system_output(
        self, timestamp: float, state_x: SystemStateX, input_u: SystemInputU, params: Any
    ) -> SystemOutputY:
        return state_x

    def make_constraints(self, x0: SystemStateX, xf: SystemStateX, goal: GoToPoseGoal) -> SystemConstraints:
        constraints = []
        constraints.append(self.make_state_constraints(goal))
        # TODO limit state
        return constraints

    def convert_to_command(self, output_vector: np.ndarray) -> Velocity:
        return Velocity(x=output_vector[0], y=output_vector[1], theta=output_vector[2])
