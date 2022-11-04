# Adapted from edu.wpi.first.math.controller.ProfiledPIDController
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.

from typing import Optional
from .pid import PIDController, input_modulus
from .trapezoid_profile import State, Constraints, TrapezoidProfile


class ProfiledPIDController(PIDController):
    def __init__(self, kp: float, ki: float, kd: float, constraints: Constraints, period: float = 0.02) -> None:
        super().__init__(kp, ki, kd, period)
        self.goal = State()
        self.trap_setpoint = State()
        self.constraints = constraints
    
    def set_goal_state(self, goal: State):
        """
        * Sets the goal for the ProfiledPIDController.
        *
        * @param goal The desired goal state.
        """
        self.goal = goal

    def set_goal(self, goal: float):
        """
        * Sets the goal for the ProfiledPIDController.
        *
        * @param goal The desired goal position.
        """
        self.goal = State(goal, 0.0)

    def at_goal(self):
        """
        * Returns true if the error is within the tolerance of the error.
        *
        * <p>This will return false until at least one input value has been computed.
        *
        * @return True if the error is within the tolerance of the error.
        """
        return self.at_setpoint() and self.goal == self.trap_setpoint
    
    def set_constraints(self, constraints: Constraints):
        """
        * Set velocity and acceleration constraints for goal.
        *
        * @param constraints Velocity and acceleration constraints for goal.
        """
        self.constraints = constraints

    def calculate(self, measurement: float, setpoint: Optional[float] = None) -> float:
        """
        * Returns the next output of the PID controller.
        *
        * @param measurement The current measurement of the process variable.
        * @return The controller's next output.
        """
        if self.is_continuous_input_enabled():
            # Get error which is smallest distance between goal and measurement
            error_bound = (self.maximum_input - self.minimum_input) / 2.0
            goal_min_distance = \
                input_modulus(self.goal.position - measurement, -error_bound, error_bound)
            setpoint_min_distance = \
                input_modulus(self.setpoint.position - measurement, -error_bound, error_bound)

            # recompute the profile goal with the smallest error, thus giving the shortest path. the goal
            # may be outside the input range after this operation, but that's ok because the controller
            # will still go there and report an error of zero. in other words, the setpoint only needs to
            # be offset from the measurement by the input range modulus they don't need to be equal.
            self.goal.position = goal_min_distance + measurement
            self.trap_setpoint.position = setpoint_min_distance + measurement

        profile = TrapezoidProfile(self.constraints, self.goal, self.trap_setpoint)
        self.trap_setpoint = profile.calculate(self.period)
        return super().calculate(measurement, self.trap_setpoint.position)

    def reset(self, measurement: State):
        """
        * Reset the previous error and the integral term.
        *
        * @param measurement The current measured State of the system.
        """
        super().reset()
        self.trap_setpoint = measurement

    def reset_position(self, measurement: float):
        """
        * Reset the previous error and the integral term.
        *
        * @param measuredPosition The current measured position of the system. The velocity is assumed to
        *     be zero.
        """
        super().reset()
        self.trap_setpoint = State(measurement, 0.0)
