# Adapted from edu.wpi.first.math.controller.PIDController
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.


import math
from typing import Optional

class PIDController:
    def __init__(self, kp: float, ki: float, kd: float, period: float = 0.02) -> None:
        """
        * Allocates a PIDController with the given constants for kp, ki, and kd and a default period of
        * 0.02 seconds.
        *
        * @param kp The proportional coefficient.
        * @param ki The integral coefficient.
        * @param kd The derivative coefficient.
        * @param period The period between controller updates in seconds. Must be non-zero and positive.
        """

        # Factor for "proportional" control
        self.kp = kp

        # Factor for "integral" control
        self.ki = ki

        # Factor for "derivative" control
        self.kd = kd

        # The period (in seconds) of the loop that calls the controller
        self.period = period
        if self.period < 0.0:
            raise ValueError(f"Controller period must be a non-zero positive number! {self.period}s")

        self.maximum_integral = 1.0
        self.minimum_integral = -1.0
        self.maximum_input = 0.0
        self.minimum_input = 0.0

        # Do the endpoints wrap around? eg. Absolute encoder
        self.continuous = False

        # The error at the time of the most recent call to calculate()
        self.position_error = 0.0
        self.velocity_error = 0.0

        # The error at the time of the second-most-recent call to calculate() (used to compute velocity)
        self.prev_error = 0.0

        # The sum of the errors for use in the integral calc
        self.total_error = 0.0

        # The error that is considered at setpoint.
        self.position_tolerance = 0.05
        self.velocity_tolerance = float('inf')

        self.setpoint = 0.0
        self.measurement = 0.0

    def set_pid(self, kp: float, ki: float, kd: float) -> None:
        """
        * Sets the PID Controller gain parameters.
        *
        * <p>Set the proportional, integral, and differential coefficients.
        *
        * @param kp The proportional coefficient.
        * @param ki The integral coefficient.
        * @param kd The derivative coefficient.
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd

    def set_p(self, kp: float) -> None:
        """
        * Sets the Proportional coefficient of the PID controller gain.
        *
        * @param kp proportional coefficient
        """
        self.kp = kp

    def set_i(self, ki: float) -> None:
        """
        * Sets the Integral coefficient of the PID controller gain.
        *
        * @param ki integral coefficient
        """
        self.ki = ki

    def set_d(self, kd: float) -> None:
        """
        * Sets the Differential coefficient of the PID controller gain.
        *
        * @param kd differential coefficient
        """
        self.kd = kd

    def set_setpoint(self, setpoint: float) -> None:
        """
        * Sets the setpoint for the PIDController.
        *
        * @param setpoint The desired setpoint.
        """
        self.setpoint = setpoint

    def at_setpoint(self) -> bool:
        """
        * Returns true if the error is within the tolerance of the setpoint.
        *
        * <p>This will return false until at least one input value has been computed.
        *
        * @return Whether the error is within the acceptable bounds.
        """
        if (self.continuous):
            error_bound = (self.maximum_input - self.minimum_input) / 2.0
            position_error = self.input_modulus(self.setpoint - self.measurement, -error_bound, error_bound)
        else:
            position_error = self.setpoint - self.measurement
        velocity_error = (position_error - self.prev_error) / self.period
        return abs(position_error) < self.position_tolerance and abs(velocity_error) < self.velocity_tolerance

    def enable_continuous_input(self, minimum_input: float, maximum_input: float) -> None:
        """
        * Enables continuous input.
        *
        * <p>Rather then using the max and min input range as constraints, it considers them to be the
        * same point and automatically calculates the shortest route to the setpoint.
        *
        * @param minimum_input The minimum value expected from the input.
        * @param maximum_input The maximum value expected from the input.
        """
        self.continuous = True
        self.minimum_input = minimum_input
        self.maximum_input = maximum_input

    def disable_continuous_input(self) -> None:
        """Disables continuous input."""
        self.continuous = False
    
    def is_continuous_input_enabled(self) -> bool:
        """
        * Returns true if continuous input is enabled.
        *
        * @return True if continuous input is enabled.
        """
        return self.continuous

    def set_integrator_range(self, minimum_integral: float, maximum_integral: float):
        """
        * Sets the minimum and maximum values for the integrator.
        *
        * <p>When the cap is reached, the integrator value is added to the controller output rather than
        * the integrator value times the integral gain.
        *
        * @param minimum_integral The minimum value of the integrator.
        * @param maximum_integral The maximum value of the integrator.
        """
        self.minimum_integral = minimum_integral
        self.maximum_integral = maximum_integral
    
    def set_tolerance(self, position_tolerance: float, velocity_tolerance: Optional[float] = None) -> None:
        """
        * Sets the error which is considered tolerable for use with atSetpoint().
        *
        * @param position_tolerance Position error which is tolerable.
        * @param velocity_tolerance Velocity error which is tolerable.
        """
        self.position_tolerance = position_tolerance
        self.velocity_tolerance = float('inf') if velocity_tolerance is None else velocity_tolerance

    def get_position_error(self) -> float:
        """
        * Returns the difference between the setpoint and the measurement.
        *
        * @return The error.
        """
        return self.position_error

    def get_velocity_error(self) -> float:
        """
        * Returns the velocity error.
        *
        * @return The velocity error.
        """
        return self.velocity_error

    def calculate(self, measurement: float, setpoint: Optional[float] = None) -> float:
        """
        * Returns the next output of the PID controller.
        *
        * @param measurement The current measurement of the process variable.
        * @param setpoint The new setpoint of the controller.
        * @return The next controller output.
        """
        if setpoint is not None:
            self.set_setpoint(setpoint)

        self.measurement = measurement
        self.prev_error = self.position_error

        if self.continuous:
            error_bound = (self.maximum_input - self.minimum_input) / 2.0
            self.position_error = self.input_modulus(self.setpoint - self.measurement, -error_bound, error_bound)
        else:
            self.position_error = self.setpoint - measurement

        self.velocity_error = (self.position_error - self.prev_error) / self.period

        if self.ki != 0:
            self.totalError = self.clamp(
                self.total_error + self.position_error * self.period,
                self.minimum_integral / self.ki,
                self.maximum_integral / self.ki
            )

        return self.kp * self.position_error + self.ki * self.total_error + self.kd * self.velocity_error

    def reset(self) -> None:
        """Resets the previous error and the integral term."""
        self.prev_error = 0.0
        self.total_error = 0.0

    @staticmethod
    def input_modulus(input: float, minimum_input: float, maximum_input: float) -> float:
        """
        * Returns modulus of input.
        *
        * @param input Input value to wrap.
        * @param minimum_input The minimum value expected from the input.
        * @param maximum_input The maximum value expected from the input.
        * @return The wrapped value.
        """
        modulus = maximum_input - minimum_input

        # Wrap input if it's above the maximum input
        num_max = int((input - minimum_input) / modulus)
        input -= num_max * modulus

        #Wrap input if it's below the minimum input
        num_min = int((input - maximum_input) / modulus)
        input -= num_min * modulus

        return input

    @staticmethod
    def clamp(value: float, low: float, high: float) -> float:
        """
        * Returns value clamped between low and high boundaries.
        *
        * @param value Value to clamp.
        * @param low The lower boundary to which to clamp value.
        * @param high The higher boundary to which to clamp value.
        * @return The clamped value.
        """
        return max(low, min(value, high))

    @staticmethod
    def clamp_region(value: float, low: Optional[float], high: Optional[float], epsilon=1E-6) -> float:
        """
        * Returns value clamped between -high..-low and low..high. If abs(value) < epsilon, return
        * zero
        *
        * @param value Value to clamp.
        * @param low The lower boundary to which to clamp value.
        * @param high The higher boundary to which to clamp value.
        * @param epsilon Basically zero value
        * @return The clamped value.
        """

        if abs(value) < epsilon:
            return 0.0
        if low is None:
            low = 0.0
        if high is None:
            high_clamped = value
        else:
            high_clamped = min(abs(value), abs(high))
        clamped = max(abs(low), high_clamped)
        return math.copysign(clamped, value)

    @staticmethod
    def apply_deadband(value, deadband, max_magnitude=1.0):
        """
        * Returns 0.0 if the given value is within the specified range around zero. The remaining range
        * between the deadband and the maximum magnitude is scaled from 0.0 to the maximum magnitude.
        *
        * @param value Value to clip.
        * @param deadband Range around zero.
        * @param max_magnitude The maximum magnitude of the input. Can be infinite.
        * @return The value after the deadband is applied.
        """
        if abs(value) > deadband:
            if max_magnitude / deadband > 1.0e12:
                # If max magnitude is sufficiently large, the implementation encounters
                # roundoff error.  Implementing the limiting behavior directly avoids
                # the problem.
                return value - deadband if value > 0.0 else value + deadband
            if value > 0.0:
                # Map deadband to 0 and map max to max.
                #
                # y - y₁ = m(x - x₁)
                # y - y₁ = (y₂ - y₁)/(x₂ - x₁) (x - x₁)
                # y = (y₂ - y₁)/(x₂ - x₁) (x - x₁) + y₁
                #
                # (x₁, y₁) = (deadband, 0) and (x₂, y₂) = (max, max).
                # x₁ = deadband
                # y₁ = 0
                # x₂ = max
                # y₂ = max
                #
                # y = (max - 0)/(max - deadband) (x - deadband) + 0
                # y = max/(max - deadband) (x - deadband)
                # y = max (x - deadband)/(max - deadband)
                return max_magnitude * (value - deadband) / (max_magnitude - deadband)
            else:
                # Map -deadband to 0 and map -max to -max.
                #
                # y - y₁ = m(x - x₁)
                # y - y₁ = (y₂ - y₁)/(x₂ - x₁) (x - x₁)
                # y = (y₂ - y₁)/(x₂ - x₁) (x - x₁) + y₁
                #
                # (x₁, y₁) = (-deadband, 0) and (x₂, y₂) = (-max, -max).
                # x₁ = -deadband
                # y₁ = 0
                # x₂ = -max
                # y₂ = -max
                #
                # y = (-max - 0)/(-max + deadband) (x + deadband) + 0
                # y = max/(max - deadband) (x + deadband)
                # y = max (x + deadband)/(max - deadband)
                return max_magnitude * (value + deadband) / (max_magnitude - deadband)
        else:
            return 0.0
