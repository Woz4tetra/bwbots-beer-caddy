#!/usr/bin/env python3
import math
from typing import Any

import control
import numpy as np
from scipy.optimize import NonlinearConstraint

from bw_navigation.base_optimizer import BaseOptimizer
from bw_navigation.optimization_helpers import (
    FullModulesCommand,
    ModuleCommand,
    SystemConstraints,
    SystemInputU,
    SystemOutputY,
    SystemStateX,
    module_system_output,
    module_system_update,
)
from bw_tools.structs.go_to_goal import GoToPoseGoal


class BwDriveTrainOptimizer(BaseOptimizer[FullModulesCommand]):
    def __init__(self) -> None:
        self.width = 0.115  # meters
        self.length = 0.160  # meters
        self.armature = 0.037  # meters

        self.min_radius_of_curvature = 0.15

        self.locations = np.array(
            [
                (-self.length / 2.0, self.width / 2.0),  # module 1, channel 0, back left
                (-self.length / 2.0, -self.width / 2.0),  # module 2, channel 1, back right
                (self.length / 2.0, self.width / 2.0),  # module 3, channel 2, front left
                (self.length / 2.0, -self.width / 2.0),  # module 4, channel 3, front right
            ]
        )
        ALCOVE_ANGLE = 0.5236  # 30 degrees
        FRONT_ANGLE = -1.2967  # -74.293 degrees
        STRAIGHT_ANGLE = 0.0
        self.min_azimuth_angles = [
            FRONT_ANGLE,  # -75 deg
            math.pi - ALCOVE_ANGLE,  # 150 deg
            FRONT_ANGLE + math.pi,  # 105 deg
            -ALCOVE_ANGLE,  # -30 deg
        ]
        self.max_azimuth_angles = [
            ALCOVE_ANGLE,  # 30 deg
            math.pi - FRONT_ANGLE,  # 225 deg
            ALCOVE_ANGLE + math.pi,  # 210 deg
            -FRONT_ANGLE,  # 75 deg
        ]
        self.azimuth_limits = np.array(list(zip(self.min_azimuth_angles, self.max_azimuth_angles)))
        self.wheel_velocity_limits = (-1.0, 1.0)  # m/s
        lower_limits = []
        upper_limits = []
        for index in range(len(self.locations)):
            lower_limits.append(self.wheel_velocity_limits[0])
            lower_limits.append(self.azimuth_limits[index][0])
            upper_limits.append(self.wheel_velocity_limits[1])
            upper_limits.append(self.azimuth_limits[index][1])
        self.lower_limits = np.array(lower_limits)
        self.upper_limits = np.array(upper_limits)

        self.neutral_azimuth_angles = [
            STRAIGHT_ANGLE,
            STRAIGHT_ANGLE + math.pi,
            STRAIGHT_ANGLE + math.pi,
            STRAIGHT_ANGLE,
        ]
        self.neutral = []
        neutral_values = []
        for index in range(len(self.locations)):
            self.neutral.append(0.0)
            self.neutral.append(self.neutral_azimuth_angles[index])
            neutral_values.append(ModuleCommand(0.0, self.neutral_azimuth_angles[index]))
        neutral_commands = FullModulesCommand(neutral_values)

        input_names = ("vx", "vy", "vt", "ax", "ay", "at")
        state_names = ("x", "y", "t", "vx", "vy", "vt")

        output_names = []
        for index in range(len(self.locations)):
            output_names.append("module_%s_wheel" % index)
            output_names.append("module_%s_azimuth" % index)
        output_names = tuple(output_names)

        system = control.NonlinearIOSystem(
            self.system_update,
            self.system_output,
            name="go_to_pose",
            states=state_names,
            inputs=input_names,
            outputs=output_names,
        )

        super().__init__(system, neutral_commands)

    def system_update(
        self, timestamp: float, state_x: SystemStateX, input_u: SystemInputU, params: Any
    ) -> SystemOutputY:
        return module_system_update(state_x, input_u)

    def system_output(
        self, timestamp: float, state_x: SystemStateX, input_u: SystemInputU, params: Any
    ) -> SystemOutputY:
        return self.system_output_wrapper(input_u)

    def system_output_wrapper(self, input_u: SystemInputU) -> SystemOutputY:
        return module_system_output(
            input_u,
            self.locations,
            self.armature,
            self.min_radius_of_curvature,
            self.azimuth_limits,
            self.wheel_velocity_limits,
            self.update_delay,
        )

    def make_output_limit_constraint(self) -> NonlinearConstraint:
        def constraint(state_x: SystemStateX, input_u: SystemInputU) -> np.ndarray:
            return self.system_output_wrapper(input_u)

        return NonlinearConstraint(
            constraint,
            self.lower_limits,
            self.upper_limits,
        )

    def make_constraints(self, x0: SystemStateX, xf: SystemStateX, goal: GoToPoseGoal) -> SystemConstraints:
        constraints = []
        constraints.append(self.make_input_constraints(goal))
        constraints.append(self.make_output_limit_constraint())
        return constraints

    def convert_to_command(self, output_vector: np.ndarray) -> FullModulesCommand:
        command_values = []
        for index in range(len(self.locations)):
            wheel_velocity = output_vector[index * 2]
            azimuth = output_vector[index * 2 + 1]
            subcommand = ModuleCommand(wheel_velocity, azimuth)
            command_values.append(subcommand)
        return FullModulesCommand(command_values)
