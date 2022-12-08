# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.


import math
from typing import List, Optional

from ..robot_state import Pose2d


def lerp(start_value, end_value, t):
    """
    * Linearly interpolates between two values.
    *
    * @param start_value The start value.
    * @param end_value The end value.
    * @param t The fraction for interpolation.
    * @return The interpolated value.
    """
    return start_value + (end_value - start_value) * t


def lerp_poses(start_value: Pose2d, end_value: Pose2d, t: float):
    """
    * Linearly interpolates between two poses.
    *
    * @param start_value The start pose.
    * @param end_value The end pose.
    * @param t The fraction for interpolation.
    * @return The interpolated pose.
    """
    return start_value.transform_by((end_value.relative_to(start_value)) * t)


class State:
    """
    * Represents a time-parameterized trajectory. The trajectory contains of various States that
    * represent the pose, curvature, time elapsed, velocity, and acceleration at that point.
    """

    def __init__(self,
            time_seconds: float = 0.0, 
            velocity: float = 0.0, 
            acceleration: float = 0.0, 
            pose: Pose2d = Pose2d(), 
            curvature: float = 0.0) -> None:
        pass
        # The time elapsed since the beginning of the trajectory.
        self.time_seconds = time_seconds

        self.velocity = velocity

        # The acceleration at that point of the trajectory.
        self.acceleration = acceleration

        # The pose at that point of the trajectory.
        self.pose = pose

        # The curvature at that point of the trajectory.
        self.curvature = curvature

    def interpolate(self, end_value: "State", i: float) -> "State":
        """
        * Interpolates between two States.
        *
        * @param end_value The end value for the interpolation.
        * @param i The interpolant (fraction).
        * @return The interpolated state.
        """
        # Find the new t value.
        new_t = lerp(self.time_seconds, end_value.time_seconds, i)

        # Find the delta time between the current state and the interpolated state.
        delta_t = new_t - self.time_seconds

        # If delta time is negative, flip the order of interpolation.
        if delta_t < 0.0:
            return end_value.interpolate(self, 1 - i)

        # Check whether the robot is reversing at this stage.
        reversing: bool = \
            self.velocity < 0 \
                or abs(self.velocity) < 1E-9 and self.acceleration < 0.0

        # Calculate the new velocity
        # v_f = v_0 + at
        new_v = self.velocity + (self.acceleration * delta_t)

        # Calculate the change in position.
        # delta_s = v_0 t + 0.5 at^2
        new_s = \
            (self.velocity * delta_t
                    + 0.5 * self.acceleration * delta_t * delta_t) \
                * (-1.0 if reversing else 1.0)

        # Return the new state. To find the new position for the new state, we need
        # to interpolate between the two endpoint poses. The fraction for
        # interpolation is the change in position (delta s) divided by the total
        # distance between the two endpoints.
        interpolation_frac = \
            new_s / end_value.pose.distance(self.pose)

        return State(
            new_t,
            new_v,
            self.acceleration,
            lerp(self.pose, end_value.pose, interpolation_frac),
            lerp(self.curvature, end_value.curvature, interpolation_frac))

    def __str__(self):
        return "State(Sec: %.2f, Vel m/s: %.2f, Accel m/s/s: %.2f, Pose: %s, Curvature: %.2f)" % (
            self.time_seconds,
            self.velocity,
            self.acceleration,
            self.pose,
            self.curvature
        )

    def __eq__(self, other):
        if self == other:
            return True
        if not isinstance(other, self.__class__):
            return False

        return other.time_seconds == self.time_seconds and \
            other.velocity == self.velocity and \
            other.acceleration == self.acceleration and \
            other.curvature == self.curvature and \
            other.pose == self.pose


class Trajectory:
    """
    * Represents a time-parameterized trajectory. The trajectory contains of various States that
    * represent the pose, curvature, time elapsed, velocity, and acceleration at that point.
    """
    def __init__(self, states: Optional[List[State]] = None) -> None:
        self.total_time_seconds = 0.0 if states is None else states[-1].time_seconds
        self.states: List[State] = [] if states is None else states

    def get_initial_pose(self) -> Pose2d:
        """
        * Returns the initial pose of the trajectory.
        *
        * @return The initial pose of the trajectory.
        """
        return self.sample(0).pose

    def get_total_time_seconds(self):
        """
        * Returns the overall duration of the trajectory.
        *
        * @return The duration of the trajectory.
        """
        return self.total_time_seconds
    
    def get_states(self) -> List[State]:
        return self.states

    def sample(self, time_seconds: float) -> State:
        """
        * Sample the trajectory at a point in time.
        *
        * @param time_seconds The point in time since the beginning of the trajectory to sample.
        * @return The state at that point in time.
        """
        if time_seconds <= self.states[0].time_seconds:
            return self.states[0]

        if time_seconds >= self.total_time_seconds:
            return self.states[len(self.states) - 1]

        # To get the element that we want, we will use a binary search algorithm
        # instead of iterating over a for-loop. A binary search is O(std::log(n))
        # whereas searching using a loop is O(n).

        # This starts at 1 because we use the previous state later on for
        # interpolation.
        low: int = 1
        high: int = len(self.states) - 1

        while low != high:
            mid: int = int((low + high) / 2)
            if self.states[mid].time_seconds < time_seconds:
                # This index and everything under it are less than the requested
                # timestamp. Therefore, we can discard them.
                low = mid + 1
            else:
                # t is at least as large as the element at this index. This means that
                # anything after it cannot be what we are looking for.
                high = mid

        # High and Low should be the same.

        # The sample's timestamp is now greater than or equal to the requested
        # timestamp. If it is greater, we need to interpolate between the
        # previous state and the current state to get the exact state that we
        # want.
        sample: State = self.states[low]
        prev_sample: State = self.states[low - 1]

        # If the difference in states is negligible, then we are spot on!
        if abs(sample.time_seconds - prev_sample.time_seconds) < 1E-9:
            return sample

        # Interpolate between the two states for the state that we want.
        return prev_sample.interpolate(
            sample,
            (time_seconds - prev_sample.time_seconds) / (sample.time_seconds - prev_sample.time_seconds))

    def transform_by(self, transform: Pose2d) -> "Trajectory":
        """
        * Transforms all poses in the trajectory by the given transform. This is useful for converting a
        * robot-relative trajectory into a field-relative trajectory. This works with respect to the
        * first pose in the trajectory.
        *
        * @param transform The transform to transform the trajectory by.
        * @return The transformed trajectory.
        """
        first_state = self.states[0]
        first_pose = first_state.pose

        # Calculate the transformed first pose.
        new_first_pose = first_pose.transform_by(transform)
        new_states: List[State] = []

        new_states.append(State(
            first_state.time_seconds,
            first_state.velocity,
            first_state.acceleration,
            new_first_pose,
            first_state.curvature
        ))

        for i in range(len(self.states)):
            state: State = self.states[i]
            # We are transforming relative to the coordinate frame of the new initial pose.
            new_states.append(State(
                state.time_seconds,
                state.velocity,
                state.acceleration,
                new_first_pose.transform_by(state.pose.relative_to(first_pose)),
                state.curvature
            ))

        return Trajectory(new_states)

    def relative_to(self, pose: Pose2d) -> "Trajectory":
        """
        * Transforms all poses in the trajectory so that they are relative to the given pose. This is
        * useful for converting a field-relative trajectory into a robot-relative trajectory.
        *
        * @param pose The pose that is the origin of the coordinate frame that the current trajectory
        *     will be transformed into.
        * @return The transformed trajectory.
        """
        return Trajectory([
            State(
                state.time_seconds,
                state.velocity,
                state.acceleration,
                state.pose.relative_to(pose),
                state.curvature
            ) for state in self.states
        ])
    
    def concatenante(self, other: "Trajectory") -> "Trajectory":
        """
        * Concatenates another trajectory to the current trajectory. The user is responsible for making
        * sure that the end pose of this trajectory and the start pose of the other trajectory match (if
        * that is the desired behavior).
        *
        * @param other The trajectory to concatenate.
        * @return The concatenated trajectory.
        """
        if len(self.states) == 0:
            return other
        if len(other.states) == 0:
            return self
        
        # Deep copy the current states.
        states = [
            State(
                state.time_seconds,
                state.velocity,
                state.acceleration,
                state.pose,
                state.curvature
            ) for state in self.states
        ]

        # Here we omit the first state of the other trajectory because we don't want
        # two time points with different states. Sample() will automatically
        # interpolate between the end of this trajectory and the second state of the
        # other trajectory.
        for i in range(1, len(other.states)):
            s = other.states[i]
            states.append(State(
                    s.time_seconds + self.total_time_seconds,
                    s.velocity,
                    s.acceleration,
                    s.pose,
                    s.curvature
                ))
        return Trajectory(states)

    def __str__(self):
        state_strs = "\n".join([str(state) for state in self.states])
        return "Trajectory - Seconds: %.2f, States:\n%s" % (self.total_time_seconds, state_strs)

    def __eq__(self, other):
        return isinstance(other, self.__class__) and self.states == other.states

