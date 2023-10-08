#!/usr/bin/env python3
from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Any, Generic, Tuple, Type, TypeVar

import control
import control.optimal as opt  # type: ignore
import numpy as np
import rospy
from scipy.interpolate import CubicSpline
from scipy.optimize import LinearConstraint

from bw_navigation.optimization_helpers import (
    NUM_INPUTS,
    NUM_STATES,
    SystemConstraints,
    SystemInputU,
    SystemInputUArray,
    SystemOutputY,
    SystemOutputYArray,
    SystemStateX,
    SystemStateXArray,
    warmup,
)
from bw_tools.robot_state import Pose2dStamped
from bw_tools.structs.go_to_goal import GoToPoseGoal


@dataclass(frozen=True)
class SolveResult:
    success: bool
    times: np.ndarray
    outputs: SystemOutputYArray


CommandType = TypeVar('CommandType')


class BaseOptimizer(ABC, Generic[CommandType]):
    def __init__(self, system: control.NonlinearIOSystem, neutral_command: CommandType) -> None:
        warmup()
        self.neutral_command = neutral_command

        self.update_delay = 1.0 / 50.0
        self.interpolation_downsample_factor = 5.0

        self.min_path_time = 1.0
        self.enable_debug_logs = False
        self.minimizer_tolerance = 1e-6
        self.minimizer_max_iterations = 1000
        self.num_samples = 10

        self.trajectory_cost = (1.0, 1.0, 1.0)
        self.termination_cost = (1.0, 1.0, 1.0)

        self.system = system

        self.solve_result = SolveResult(
            False,
            np.array([], dtype=np.float64),
            np.array([], dtype=np.float64),
        )
        self.action_start_time = 0.0
        self.time_advancing_index = 0

        super().__init__()

    @abstractmethod
    def system_update(
        self, timestamp: float, state_x: SystemStateX, input_u: SystemInputU, params: Any
    ) -> SystemOutputY:
        pass

    @abstractmethod
    def system_output(
        self, timestamp: float, state_x: SystemStateX, input_u: SystemInputU, params: Any
    ) -> SystemOutputY:
        pass

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
        inputs_guess[0] += np.random.normal(0.0, 0.01, size=num_samples) + goal.reference_linear_speed
        inputs_guess[1] += np.random.normal(0.0, 0.01, size=num_samples) + goal.reference_linear_speed
        inputs_guess[2] += np.random.normal(0.0, 0.01, size=num_samples) + goal.reference_angular_speed
        return states_guess, inputs_guess

    def find_time_window(self, guess: Tuple[SystemStateXArray, SystemInputUArray], goal: GoToPoseGoal) -> float:
        states = guess[0]
        delta_distances = np.linalg.norm(np.diff(states, axis=1), axis=0)
        path_distance = np.sum(delta_distances)
        avg_speed = goal.reference_linear_speed
        total_time = float(path_distance / avg_speed)
        return total_time

    def make_state_constraints(
        self, goal: GoToPoseGoal
    ) -> Tuple[Type[LinearConstraint], np.ndarray, Tuple[float, ...], Tuple[float, ...]]:
        max_speed = goal.reference_linear_speed
        max_angular = goal.reference_angular_speed

        return opt.input_range_constraint(
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

    @abstractmethod
    def make_constraints(self, x0: SystemStateX, xf: SystemStateX, goal: GoToPoseGoal) -> SystemConstraints:
        pass

    def interpolate_results(
        self,
        solve_result: opt.OptimalControlResult,
    ) -> Tuple[np.ndarray, SystemOutputYArray]:
        times = solve_result.time
        total_time = times[-1] - times[0]
        assert solve_result.states is not None
        inputs_interp = CubicSpline(times, solve_result.inputs.T, bc_type='clamped')
        states_interp = CubicSpline(times, solve_result.states.T, bc_type='clamped')
        time_samples = np.linspace(
            times[0], times[-1], int(total_time / (self.update_delay * self.interpolation_downsample_factor))
        )
        inputs_samples = np.array(inputs_interp(time_samples), dtype=np.float64)
        states_samples = np.array(states_interp(time_samples), dtype=np.float64)
        outputs_samples = []
        for index in range(len(time_samples)):
            timestamp = time_samples[index]
            input_sample = inputs_samples[index, :]
            state_sample = states_samples[index, :]
            outputs_samples.append(self.system_output(timestamp, state_sample, input_sample, None))
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

    @abstractmethod
    def convert_to_command(self, output_vector: np.ndarray) -> CommandType:
        pass

    def update(self, robot_state: Pose2dStamped) -> Tuple[CommandType, bool]:
        if not self.solve_result.success:
            rospy.logerr(f"Solve result is not valid: {self.solve_result}")
            return self.neutral_command, True
        relative_time = robot_state.header.stamp - self.action_start_time
        if relative_time > self.solve_result.times[-1]:
            return self.neutral_command, True
        index = self.get_nearest_index(relative_time, self.solve_result.times)
        output = self.solve_result.outputs[:, index]
        command = self.convert_to_command(output)
        return command, False
