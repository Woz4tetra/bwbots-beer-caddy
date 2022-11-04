# Adapted from edu.wpi.first.math.controller.HolonomicDriveController
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.

import math
from typing import Optional
from ..robot_state import Pose2d, Velocity
from .pid import PIDController
from .profiled_pid import ProfiledPIDController
from .trapezoid_profile import State
from .controller import Controller


class HolonomicDriveController(Controller):
    """
    * This holonomic drive controller can be used to follow trajectories using a holonomic drivetrain
    * (i.e. swerve or mecanum). Holonomic trajectory following is a much simpler problem to solve
    * compared to skid-steer style drivetrains because it is possible to individually control forward,
    * sideways, and angular velocity.
    *
    * <p>The holonomic drive controller takes in one PID controller for each direction, forward and
    * sideways, and one profiled PID controller for the angular direction. Because the heading dynamics
    * are decoupled from translations, users can specify a custom heading that the drivetrain should
    * point toward. This heading reference is profiled for smoothness.
    """

    def __init__(self, x_controller: PIDController, y_controller: PIDController, theta_controller: ProfiledPIDController) -> None:
        """
        * Constructs a holonomic drive controller.
        *
        * @param xController A PID Controller to respond to error in the field-relative x direction.
        * @param yController A PID Controller to respond to error in the field-relative y direction.
        * @param thetaController A profiled PID controller to respond to error in angle.
        """
        super().__init__()

        self.x_controller = x_controller
        self.y_controller = y_controller
        self.theta_controller = theta_controller

        self.first_run = True

    def calculate(self, **kwargs) -> Velocity:
        current_pose: Pose2d = kwargs["current_pose"]
        pose_ref: Optional[Pose2d] = kwargs.get("pose_ref", None)
        linear_velocity_ref_meters: Optional[float] = kwargs.get("linear_velocity_ref_meters", None)
        angle_ref: Optional[float] = kwargs.get("angle_ref", None)
        desired_state: Optional[State] = kwargs.get("desired_state", None)

        if pose_ref is not None:
            return self.calculate_from_poses(current_pose, pose_ref, linear_velocity_ref_meters, angle_ref)
        elif desired_state is not None:
            return self.calculate_from_state(current_pose, desired_state, angle_ref)
        else:
            raise ValueError(f"Invalid parameters for {self.__class__.__name__}.calculate: {kwargs}")
    
    def calculate_from_poses(self, current_pose: Pose2d,
            pose_ref: Pose2d,
            linear_velocity_ref_meters: float,
            angle_ref: Optional[float] = None) -> Velocity:
        """
        * Returns the next output of the holonomic drive controller.
        *
        * @param current_pose The current pose.
        * @param pose_ref The desired pose.
        * @param linear_velocity_ref_meters The linear velocity reference.
        * @param angle_ref The angular reference. If None, pose_ref.theta is used.
        * @return The next output of the holonomic drive controller.
        """
        current_pose: Pose2d
        pose_ref: Pose2d
        linear_velocity_ref_meters: float
        angle_ref: Optional[float] = None

        # If this is the first run, then we need to reset the theta controller to the current pose's
        # heading.
        if self.first_run:
            self.theta_controller.reset(current_pose.theta)
            self.first_run = False

        if angle_ref is None:
            angle_ref = pose_ref.theta

        # calculate feedforward velocities (odom-relative).
        x_ff = linear_velocity_ref_meters * math.cos(pose_ref.theta)
        y_ff = linear_velocity_ref_meters * math.sin(pose_ref.theta)
        theta_ff = \
            self.theta_controller.calculate(current_pose, angle_ref)

        self.pose_error = pose_ref.relative_to(current_pose)
        self.rotation_error = angle_ref - current_pose.theta

        if self.enabled:
            return Velocity.from_global_relative_speeds(x_ff, y_ff, theta_ff, current_pose.theta)

        # calculate feedback velocities (based on position error).
        x_feedback = self.x_controller.calculate(current_pose.x, pose_ref.x)
        y_feedback = self.y_controller.calculate(current_pose.y, pose_ref.y)

        # return next output.
        return Velocity.from_global_relative_speeds(
            x_ff + x_feedback, y_ff + y_feedback, theta_ff, current_pose.theta)

    def calculate_from_state(
        self, current_pose: Pose2d, desired_state: State, angle_ref: Optional[float]) -> Velocity:
        """
        * Returns the next output of the holonomic drive controller.
        *
        * @param currentPose The current pose.
        * @param desiredState The desired trajectory state.
        * @param angleRef The desired end-angle.
        * @return The next output of the holonomic drive controller.
        """
        return self.calculate(
            current_pose, desired_state.pose_meters, desired_state.velocity_meters_per_second, angle_ref)
