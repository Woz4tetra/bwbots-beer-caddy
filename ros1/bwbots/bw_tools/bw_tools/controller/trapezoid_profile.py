# Adapted from edu.wpi.first.math.trajectory.TrapezoidProfile
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.

import math
from dataclasses import dataclass


@dataclass
class State:
    position: float = 0.0
    velocity: float = 0.0

@dataclass
class Constraints:
    max_velocity: float = 0.0
    max_acceleration: float = 0.0



class TrapezoidProfile:
    """
     * A trapezoid-shaped velocity profile.
    *
    * <p>While this class can be used for a profiled movement from start to finish, the intended usage
    * is to filter a reference's dynamics based on trapezoidal velocity constraints. To compute the
    * reference obeying this constraint, do the following.
    *
    * <p>Initialization:
    *
    * <pre><code>
    * constraints = trapezoid_profile.Constraints(kMaxV, kMaxA)
    * previousProfiledReference = trapezoid_profile.State(initialReference, 0.0)
    * </code></pre>
    *
    * <p>Run on update:
    *
    * <pre><code>
    * profile = trapezoid_profile.TrapezoidProfile(constraints, unprofiledReference, previousProfiledReference)
    * previousProfiledReference = profile.calculate(timeSincePreviousUpdate)
    * </code></pre>
    *
    * <p>where `unprofiledReference` is free to change between calls. Note that when the unprofiled
    * reference is within the constraints, `calculate()` returns the unprofiled reference unchanged.
    *
    * <p>Otherwise, a timer can be started to provide monotonic values for `calculate()` and to
    * determine when the profile has completed via `isFinished()`.
    """
    def __init__(self, constraints: Constraints, goal: State, initial: State = State(0.0, 0.0)) -> None:
        """
        * Construct a TrapezoidProfile.
        *
        * @param constraints The constraints on the profile, like maximum velocity.
        * @param goal The desired state when the profile is complete.
        * @param initial The initial state (usually the current state).
        """
        # The direction of the profile, either 1 for forwards or -1 for inverted
        self.direction: int = -1 if self.should_flip_acceleration(initial, goal) else 1

        self.constraints: Constraints = constraints
        self.initial: State = self.direct(initial)
        self.goal: State = self.direct(goal)

        if self.initial.velocity > self.constraints.max_velocity:
            self.initial.velocity = self.constraints.max_velocity

        # Deal with a possibly truncated motion profile (with nonzero initial or
        # final velocity) by calculating the parameters as if the profile began and
        # ended at zero velocity
        cutoff_begin = self.initial.velocity / self.constraints.max_acceleration
        cutoff_dist_begin = cutoff_begin * cutoff_begin * self.constraints.max_acceleration / 2.0

        cutoff_end = self.goal.velocity / self.constraints.max_acceleration
        cutoff_dist_end = cutoff_end * cutoff_end * self.constraints.max_acceleration / 2.0

        # Now we can calculate the parameters as if it was a full trapezoid instead
        # of a truncated one
        full_trapezoid_dist = \
            cutoff_dist_begin + (self.goal.position - self.initial.position) + cutoff_dist_end
        acceleration_time = self.constraints.max_velocity / self.constraints.max_acceleration

        full_speed_dist = \
            full_trapezoid_dist - acceleration_time * acceleration_time * self.constraints.max_acceleration
        
        # handle the case where the profile never reaches full speed
        if full_speed_dist < 0:
            acceleration_time = math.sqrt(full_trapezoid_dist / self.constraints.max_acceleration)
            full_speed_dist = 0

        self.end_accel = acceleration_time - cutoff_begin
        self.end_full_speed = self.end_accel + full_speed_dist / self.constraints.max_velocity
        self.end_decel = self.end_full_speed + acceleration_time - cutoff_end


    def should_flip_acceleration(initial: State, goal: State) -> bool:
        """
        * Returns true if the profile inverted.
        *
        * <p>The profile is inverted if goal position is less than the initial position.
        *
        * @param initial The initial state (usually the current state).
        * @param goal The desired state when the profile is complete.
        """
        return initial.position > goal.position

    def direct(self, in_state: State) -> State:
        """Flip the sign of the velocity and position if the profile is inverted"""
        result = State(in_state.position, in_state.velocity)
        result.position = result.position * self.direction
        result.velocity = result.velocity * self.direction
        return result

    def calculate(self, timestamp) -> float:
        """
        * Calculate the correct position and velocity for the profile at a time t where the beginning of
        * the profile was at time t = 0.
        *
        * @param timestamp The time since the beginning of the profile.
        * @return The position and velocity of the profile at time t.
        """
        result = State(self.initial.position, self.initial.velocity)

        if timestamp < self.end_accel:
            result.velocity += timestamp * self.constraints.max_acceleration
            result.position += (self.initial.velocity + timestamp * self.constraints.max_acceleration / 2.0) * timestamp
        elif timestamp < self.end_full_speed:
            result.velocity = self.constraints.max_velocity
            result.position += \
                (self.initial.velocity + self.end_accel * self.constraints.max_acceleration / 2.0) * self.end_accel \
                    + self.constraints.max_velocity * (timestamp - self.end_accel)
        elif timestamp <= self.end_decel:
            result.velocity = self.goal.velocity + (self.end_decel - timestamp) * self.constraints.max_acceleration
            time_left = self.end_decel - timestamp
            result.position = \
                self.goal.position \
                    - (self.goal.velocity + time_left * self.constraints.max_acceleration / 2.0) * time_left
        else:
            result = self.goal

        return self.direct(result)

    def time_left_until(self, target: float) -> float:
        """
        * Returns the time left until a target distance in the profile is reached.
        *
        * @param target The target distance.
        * @return The time left until a target distance in the profile is reached.
        """
        position = self.initial.position * self.direction
        velocity = self.initial.velocity * self.direction

        end_accel = self.end_accel * self.direction
        end_full_speed = self.end_full_speed * self.direction - end_accel

        if target < position:
            end_accel = -end_accel
            end_full_speed = -end_full_speed
            velocity = -velocity

        end_accel = max(end_accel, 0.0)
        end_full_speed = max(end_full_speed, 0.0)

        acceleration = self.constraints.max_acceleration
        deceleration = -self.constraints.max_acceleration

        dist_to_target = abs(target - position)
        if dist_to_target < 1e-6:
            return 0.0

        accel_dist = velocity * end_accel + 0.5 * acceleration * end_accel * end_accel

        if end_accel > 0:
            decel_velocity = math.sqrt(abs(velocity * velocity + 2 * acceleration * accel_dist))
        else:
            decel_velocity = velocity

        full_speed_dist = self.constraints.max_velocity * end_full_speed

        if accel_dist > dist_to_target:
            accel_dist = dist_to_target
            full_speed_dist = 0.0
            decel_dist = 0.0
        elif accel_dist + full_speed_dist > dist_to_target:
            full_speed_dist = dist_to_target - accel_dist
            decel_dist = 0.0
        else:
            decel_dist = dist_to_target - full_speed_dist - accel_dist

        accel_time = \
            (-velocity + math.sqrt(math.abs(velocity * velocity + 2 * acceleration * accel_dist))) \
                / acceleration

        decel_time = \
            (-decel_velocity
                    + math.sqrt(
                        math.abs(decel_velocity * decel_velocity + 2 * deceleration * decel_dist))) \
                / deceleration

        full_speed_time = full_speed_dist / self.constraints.max_velocity

        return accel_time + full_speed_time + decel_time

    def total_time(self) -> float:
        """
        * Returns the total time the profile takes to reach the goal.
        *
        * @return The total time the profile takes to reach the goal.
        """
        return self.end_decel

    def isFinished(self, timestamp: float) -> bool:
        """
        * Returns true if the profile has reached the goal.
        *
        * <p>The profile has reached the goal if the time since the profile started has exceeded the
        * profile's total time.
        *
        * @param t The time since the beginning of the profile.
        * @return True if the profile has reached the goal.
        """
        return timestamp >= self.total_time()
