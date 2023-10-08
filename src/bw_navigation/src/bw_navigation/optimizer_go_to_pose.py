#!/usr/bin/env python3
import math
from typing import Any, Optional, Tuple

import control
import control.optimal as opt  # type: ignore
import numpy as np
import rospy

from bw_navigation.navigation_action import NavigationAction
from bw_navigation.optimization_helpers import (
    NUM_INPUTS,
    NUM_STATES,
    SystemConstraints,
    SystemInputU,
    SystemInputUArray,
    SystemStateX,
    SystemStateXArray,
    SystemStateXdot,
    compute_module_inputs,
)
from bw_tools.robot_state import Pose2dStamped, Velocity
from bw_tools.structs.go_to_goal import GoToPoseGoal


class OptimizerGoToPose(NavigationAction):
    def __init__(self) -> None:
        rospy.init_node(
            "optimizer_go_to_pose",
            log_level=rospy.DEBUG,
        )

        self.width = 0.115  # meters
        self.length = 0.160  # meters
        self.armature = 0.037  # meters

        self.update_delay = 1.0 / 50.0
        self.locations = [
            (-self.length / 2.0, self.width / 2.0),  # module 1, channel 0, back left
            (-self.length / 2.0, -self.width / 2.0),  # module 2, channel 1, back right
            (self.length / 2.0, self.width / 2.0),  # module 3, channel 2, front left
            (self.length / 2.0, -self.width / 2.0),  # module 4, channel 3, front right
        ]
        ALCOVE_ANGLE = 0.5236  # 30 degrees
        FRONT_ANGLE = -1.2967  # -74.293 degrees
        self.min_module_angles = [
            FRONT_ANGLE,  # -75 deg
            math.pi - ALCOVE_ANGLE,  # 150 deg
            FRONT_ANGLE + math.pi,  # 105 deg
            -ALCOVE_ANGLE,  # -30 deg
        ]

        self.min_radius_of_curvature = 0.15
        self.min_path_time = 1.0
        self.enable_debug_logs = False
        self.minimizer_tolerance = 1e-6
        self.minimizer_max_iterations = 1000

        self.trajectory_cost = []
        for index in range(len(self.locations)):
            self.trajectory_cost.append(10.0)  # wheel velocity cost
            self.trajectory_cost.append(0.1)  # azimuth cost
        self.termination_cost = (1.0, 1.0, 1.0)  # chassis velocity termination cost

        state_names = []
        for index in range(len(self.locations)):
            state_names.append("module_%s_wheel" % index)
            state_names.append("module_%s_azimuth" % index)
        state_names = tuple(state_names)
        input_names = ("vx", "vy", "vt")

        self.system = control.NonlinearIOSystem(
            self.system_update,
            self.system_output,
            name="go_to_pose",
            states=len(state_names),
            inputs=len(input_names),
            outputs=state_names,
        )

        super().__init__()

    def system_update(
        self, timestamp: float, state_x: SystemStateX, input_u: SystemInputU, params: Any
    ) -> SystemStateXdot:
        chassis_velocities = input_u[0], input_u[1], input_u[2]
        assert len(state_x) == NUM_STATES
        assert len(input_u) == NUM_INPUTS
        module_states = compute_module_inputs(
            self.locations,
            self.armature,
            state_x.tolist(),
            chassis_velocities,
        )
        return np.array(module_states, dtype=np.float64)

    def system_output(
        self, timestamp: float, state_x: SystemStateX, input_u: SystemInputU, params: Any
    ) -> SystemStateX:
        return state_x

    def generate_guess(self, goal: GoToPoseGoal) -> Tuple[SystemStateXArray, SystemInputUArray]:
        pass

    def find_time_window(self, guess: Tuple[SystemStateXArray, SystemInputUArray]) -> float:
        pass

    def make_constraints(self, x0: SystemStateX, xf: SystemStateX) -> SystemConstraints:
        pass

    def controller_init(self, goal: GoToPoseGoal):
        guess = self.generate_guess(goal)
        num_samples = len(guess[0][0])
        guess_time_window = self.find_time_window(guess)
        guess_time_window += goal.timeout
        time_window = max(self.min_path_time, guess_time_window)
        time_horizon = np.linspace(0.0, time_window, num_samples)

        traj_cost = opt.quadratic_cost(self.system, None, np.diag(self.trajectory_cost), u0=uf)
        term_cost = opt.quadratic_cost(
            self.system,
            np.diag(self.termination_cost),
            None,
            x0=guess[0][:, -1],
        )
        self.constraints = self.make_constraints(x0, xf)

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

    def controller_update(self, goal: GoToPoseGoal, robot_state: Pose2dStamped) -> Tuple[Velocity, bool]:
        return velocity_command, is_done


def main():
    node = OptimizerGoToPose()
    node.run()


if __name__ == "__main__":
    main()
