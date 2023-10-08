#!/usr/bin/env python3
import math
from typing import Any

import control
import numpy as np
from scipy.optimize import NonlinearConstraint

from bw_navigation.base_optimizer import BaseOptimizer
from bw_navigation.optimization_helpers import (
    SystemConstraints,
    SystemInputU,
    SystemOutputY,
    SystemStateX,
    module_system_update,
    strafe_constraint,
)
from bw_tools.robot_state import Velocity
from bw_tools.structs.go_to_goal import GoToPoseGoal


class ChassisOptimizer(BaseOptimizer[Velocity]):
    def __init__(self) -> None:
        input_names = ("vx", "vy", "vt", "ax", "ay", "at")
        state_names = ("x", "y", "t", "vx", "vy", "vt")

        self.lower_strafe_limit = -math.pi / 4
        self.upper_strafe_limit = math.pi / 4

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

    def make_output_limit_constraint(self) -> NonlinearConstraint:
        return NonlinearConstraint(
            strafe_constraint,
            self.lower_strafe_limit,
            self.upper_strafe_limit,
        )

    def make_constraints(self, x0: SystemStateX, xf: SystemStateX, goal: GoToPoseGoal) -> SystemConstraints:
        constraints = []
        constraints.append(self.make_input_constraints(goal))
        constraints.append(self.make_output_limit_constraint())
        return constraints

    def convert_to_command(self, output_vector: np.ndarray) -> Velocity:
        return Velocity(x=output_vector[3], y=output_vector[4], theta=output_vector[5])  # type: ignore
