#!/usr/bin/env python3
import math
from dataclasses import dataclass
from typing import Any, Tuple

import control
import control.optimal as opt  # type: ignore
import numpy as np
import rospy
from scipy.interpolate import CubicSpline
from scipy.optimize import NonlinearConstraint

from bw_navigation.optimization_helpers import (
    NUM_INPUTS,
    NUM_STATES,
    FullModulesCommand,
    ModuleCommand,
    SystemConstraints,
    SystemInputU,
    SystemInputUArray,
    SystemOutputY,
    SystemOutputYArray,
    SystemStateX,
    SystemStateXArray,
    system_output,
    system_update,
    warmup,
)
from bw_tools.robot_state import Pose2dStamped
from bw_tools.structs.go_to_goal import GoToPoseGoal


@dataclass(frozen=True)
class SolveResult:
    success: bool
    times: np.ndarray
    outputs: SystemOutputYArray


class BwDriveTrainOptimizer:
    def __init__(self) -> None:
        warmup()
        rospy.init_node(
            "optimizer_go_to_pose",
            log_level=rospy.DEBUG,
        )

        self.width = 0.115  # meters
        self.length = 0.160  # meters
        self.armature = 0.037  # meters

        self.update_delay = 1.0 / 50.0
        self.interpolation_downsample_factor = 5.0
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
        neutral_commands = []
        for index in range(len(self.locations)):
            self.neutral.append(0.0)
            self.neutral.append(self.neutral_azimuth_angles[index])
            neutral_commands.append(ModuleCommand(0.0, self.neutral_azimuth_angles[index]))
        self.neutral_commands = FullModulesCommand(neutral_commands)

        self.min_radius_of_curvature = 0.15
        self.min_path_time = 1.0
        self.enable_debug_logs = False
        self.minimizer_tolerance = 1e-6
        self.minimizer_max_iterations = 1000
        self.num_samples = 10

        self.trajectory_cost = (1.0, 1.0, 1.0)
        self.termination_cost = (1.0, 1.0, 1.0)

        input_names = ("vx", "vy", "vt")
        state_names = ("x", "y", "t")

        output_names = []
        for index in range(len(self.locations)):
            output_names.append("module_%s_wheel" % index)
            output_names.append("module_%s_azimuth" % index)
        output_names = tuple(output_names)

        self.system = control.NonlinearIOSystem(
            self.system_update,
            self.system_output,
            name="go_to_pose",
            states=state_names,
            inputs=input_names,
            outputs=output_names,
        )

        self.solve_result = SolveResult(
            False,
            np.array([], dtype=np.float64),
            np.array([], dtype=np.float64),
        )
        self.action_start_time = 0.0
        self.time_advancing_index = 0

        super().__init__()

    def system_update(
        self, timestamp: float, state_x: SystemStateX, input_u: SystemInputU, params: Any
    ) -> SystemOutputY:
        return system_update(state_x, input_u)

    def system_output(
        self, timestamp: float, state_x: SystemStateX, input_u: SystemInputU, params: Any
    ) -> SystemOutputY:
        return self.system_output_wrapper(input_u)

    def system_output_wrapper(self, input_u: SystemInputU) -> SystemOutputY:
        return system_output(
            input_u,
            self.locations,
            self.armature,
            self.min_radius_of_curvature,
            self.azimuth_limits,
            self.wheel_velocity_limits,
            self.update_delay,
        )

    def generate_guess(
        self,
        u0: SystemInputU,
        uf: SystemInputU,
        x0: SystemStateX,
        xf: SystemStateX,
        num_samples: int,
        goal: GoToPoseGoal,
    ) -> Tuple[SystemStateXArray, SystemInputUArray]:
        states_guess = np.ascontiguousarray(np.linspace(x0, xf, num_samples).T)
        inputs_guess = np.ascontiguousarray(np.linspace(u0, uf, num_samples).T)
        inputs_guess[0] += np.random.normal(0.0, goal.reference_linear_speed, size=num_samples)
        inputs_guess[1] += np.random.normal(0.0, goal.reference_linear_speed, size=num_samples)
        inputs_guess[2] += np.random.normal(0.0, goal.reference_angular_speed, size=num_samples)
        return states_guess, inputs_guess

    def find_time_window(self, guess: Tuple[SystemStateXArray, SystemInputUArray], goal: GoToPoseGoal) -> float:
        states = guess[0]
        delta_distances = np.linalg.norm(np.diff(states, axis=1), axis=0)
        path_distance = np.sum(delta_distances)
        avg_speed = goal.reference_linear_speed
        total_time = float(path_distance / avg_speed)
        return total_time

    def make_output_limit_constraint(self) -> NonlinearConstraint:
        def constraint(state_x: SystemStateX, input_u: SystemInputU) -> np.ndarray:
            return self.system_output_wrapper(input_u)

        return NonlinearConstraint(
            constraint,
            self.lower_limits,
            self.upper_limits,
        )

    def make_constraints(self, x0: SystemStateX, xf: SystemStateX, goal: GoToPoseGoal) -> SystemConstraints:
        max_speed = goal.reference_linear_speed
        max_angular = goal.reference_angular_speed

        constraints = []
        constraints.append(
            opt.input_range_constraint(
                self.system,
                [
                    -max_speed,
                    -max_speed,
                    -max_angular,
                ],
                [
                    max_speed,
                    max_speed,
                    max_angular,
                ],
            )
        )

        constraints.append(self.make_output_limit_constraint())

        return constraints

    def interpolate_results(
        self,
        solve_result: opt.OptimalControlResult,
    ) -> Tuple[np.ndarray, SystemOutputYArray]:
        times = solve_result.time
        total_time = times[-1] - times[0]
        inputs_interp = CubicSpline(times, solve_result.inputs.T, bc_type='clamped')
        time_samples = np.linspace(
            times[0], times[-1], int(total_time / (self.update_delay * self.interpolation_downsample_factor))
        )
        inputs_samples = np.array(inputs_interp(time_samples), dtype=np.float64)
        outputs_samples = []
        for index in range(inputs_samples.shape[0]):
            outputs_samples.append(self.system_output_wrapper(inputs_samples[index, :]))
        outputs_samples = np.ascontiguousarray(np.array(outputs_samples, dtype=np.float64).T)
        return time_samples, outputs_samples

    def get_nearest_index(self, time: float, times: np.ndarray) -> int:
        search_start = self.time_advancing_index
        search_end = len(times)
        for index in range(search_start, search_end):
            if times[index] >= time:
                self.time_advancing_index = index
                return index
        self.time_advancing_index = search_end - 1
        return self.time_advancing_index

    def solve(self, goal: GoToPoseGoal, robot_state: Pose2dStamped) -> bool:
        self.action_start_time = robot_state.header.stamp

        relative_goal = goal.goal.pose.relative_to(robot_state.pose)
        u0 = np.zeros(NUM_INPUTS, dtype=np.float64)
        uf = np.zeros(NUM_INPUTS, dtype=np.float64)
        x0 = np.zeros(NUM_STATES, dtype=np.float64)
        xf = np.array(relative_goal.to_list(), dtype=np.float64)
        assert len(xf) == NUM_STATES, xf

        guess = self.generate_guess(u0, uf, x0, xf, self.num_samples, goal)
        num_samples = len(guess[0][0])
        guess_time_window = self.find_time_window(guess, goal)
        guess_time_window += goal.timeout
        time_window = max(self.min_path_time, guess_time_window)
        time_horizon = np.linspace(0.0, time_window, num_samples)

        traj_cost = opt.quadratic_cost(
            self.system,
            None,
            np.diag(self.trajectory_cost),
            u0=uf,  # type: ignore
        )
        term_cost = opt.quadratic_cost(
            self.system,
            np.diag(self.termination_cost),
            None,
            x0=guess[0][:, -1],  # type: ignore
        )
        self.constraints = self.make_constraints(x0, xf, goal)

        solve_result = opt.solve_ocp(
            self.system,
            time_horizon,
            x0,
            traj_cost,
            self.constraints,
            terminal_cost=term_cost,
            initial_guess=guess,
            log=False,
            print_summary=self.enable_debug_logs,
            minimize_method='SLSQP',
            minimize_options={
                'ftol': self.minimizer_tolerance,
                'maxiter': self.minimizer_max_iterations,
            },
        )

        if not solve_result.success:
            return False

        interp_times, outputs = self.interpolate_results(solve_result)
        self.solve_result = SolveResult(
            solve_result.success,
            interp_times,
            outputs,
        )

        return True

    def update(self, goal: GoToPoseGoal, robot_state: Pose2dStamped) -> Tuple[FullModulesCommand, bool]:
        if not self.solve_result.success:
            rospy.logerr(f"Solve result is not valid: {self.solve_result}")
            return self.neutral_commands, True
        relative_time = robot_state.header.stamp - self.action_start_time
        if relative_time > self.solve_result.times[-1]:
            return self.neutral_commands, True
        index = self.get_nearest_index(relative_time, self.solve_result.times)
        output = self.solve_result.outputs[:, index]
        command_values = []
        for index in range(len(self.locations)):
            wheel_velocity = output[index * 2]
            azimuth = output[index * 2 + 1]
            subcommand = ModuleCommand(wheel_velocity, azimuth)
            command_values.append(subcommand)
        command = FullModulesCommand(command_values)
        return command, False
