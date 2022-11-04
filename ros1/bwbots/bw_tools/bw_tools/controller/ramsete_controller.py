# Adapted from edu.wpi.first.math.controller.RamseteController
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.

import math
from typing import Optional
from ..robot_state import Pose2d, Velocity
from .controller import Controller
from .trajectory import State


def sinc(x):
    """
    * Returns sin(x) / x.
    *
    * @param x Value of which to take sinc(x).
    """
    if abs(x) < 1e-9:
        return 1.0 - 1.0 / 6.0 * x * x
    else:
        return math.sin(x) / x


class RamseteController(Controller):
    """
    * Ramsete is a nonlinear time-varying feedback controller for unicycle models that drives the model
    * to a desired pose along a two-dimensional trajectory. Why would we need a nonlinear control law
    * in addition to the linear ones we have used so far like PID? If we use the original approach with
    * PID controllers for left and right position and velocity states, the controllers only deal with
    * the local pose. If the robot deviates from the path, there is no way for the controllers to
    * correct and the robot may not reach the desired global pose. This is due to multiple endpoints
    * existing for the robot which have the same encoder path arc lengths.
    *
    * <p>Instead of using wheel path arc lengths (which are in the robot's local coordinate frame),
    * nonlinear controllers like pure pursuit and Ramsete use global pose. The controller uses this
    * extra information to guide a linear reference tracker like the PID controllers back in by
    * adjusting the references of the PID controllers.
    *
    * <p>The paper "Control of Wheeled Mobile Robots: An Experimental Overview" describes a nonlinear
    * controller for a wheeled vehicle with unicycle-like kinematics a global pose consisting of x, y,
    * and theta and a desired pose consisting of x_d, y_d, and theta_d. We call it Ramsete because
    * that's the acronym for the title of the book it came from in Italian ("Robotica Articolata e
    * Mobile per i SErvizi e le TEcnologie").
    *
    * <p>See <a href="https:#file.tavsys.net/control/controls-engineering-in-frc.pdf">Controls
    * Engineering in the FIRST Robotics Competition</a> section on Ramsete unicycle controller for a
    * derivation and analysis.
    """
    def __init__(self, b=2.0, zeta=0.7) -> None:
        """
        * Construct a Ramsete unicycle controller.
        * The default arguments for b and zeta of 2.0 rad²/m²
        * and 0.7 rad⁻¹ have been well-tested to produce desirable results.
        *
        * @param b Tuning parameter (b &gt 0 rad²/m²) for which larger values make convergence more
        *     aggressive like a proportional term.
        * @param zeta Tuning parameter (0 rad⁻¹ &lt zeta &lt 1 rad⁻¹) for which larger values provide
        *     more damping in response.
        """
        super().__init__()

        self.b = b
        self.zeta = zeta

    def calculate(self, /, **kwargs) -> Velocity:
        current_pose: Pose2d = kwargs["current_pose"]
        pose_ref: Optional[Pose2d] = kwargs.get("pose_ref", None)
        linear_velocity_ref: Optional[float] = kwargs.get("linear_velocity_ref", None)
        angular_velocity_ref: Optional[float] = kwargs.get("angular_velocity_ref", None)
        desired_state: Optional[State] = kwargs.get("desired_state", None)

        if pose_ref is not None and linear_velocity_ref is not None and angular_velocity_ref is not None:
            return self.calculate_from_poses(current_pose, pose_ref, linear_velocity_ref, angular_velocity_ref)
        elif desired_state is not None:
            return self.calculate_from_state(current_pose, desired_state)
        else:
            raise ValueError(f"Invalid parameters for {self.__class__.__name__}.calculate: {kwargs}")

    def calculate_from_poses(self,
            current_pose: Pose2d,
            pose_ref: Pose2d,
            linear_velocity_ref: float,
            angular_velocity_ref: float):
        """
        * Returns the next output of the Ramsete controller.
        *
        * <p>The reference pose, linear velocity, and angular velocity should come from a drivetrain
        * trajectory.
        *
        * @param current_pose The current pose.
        * @param pose_ref The desired pose.
        * @param linear_velocity_ref The desired linear velocity in meters per second.
        * @param angular_velocity_ref The desired angular velocity in radians per second.
        * @return The next controller output.
        """

        if not self.enabled:
            return Velocity(linear_velocity_ref, 0.0, angular_velocity_ref)

        self.pose_error = pose_ref.relative_to(current_pose)

        # Aliases for equation readability
        eX = self.pose_error.x
        eY = self.pose_error.y
        eTheta = self.pose_error.theta
        vRef = linear_velocity_ref
        omegaRef = angular_velocity_ref

        k = 2.0 * self.zeta * math.sqrt(omegaRef * omegaRef + self.b * vRef * vRef)

        return Velocity(
            vRef * math.cos(self.pose_error.theta) + k * eX,
            0.0,
            omegaRef + k * eTheta + self.b * vRef * sinc(eTheta) * eY
        )

    def calculate_from_state(
        self, current_pose: Pose2d, desired_state: State) -> Velocity:
        """
        * Returns the next output of the Ramsete controller.
        *
        * <p>The reference pose, linear velocity, and angular velocity should come from a drivetrain
        * trajectory.
        *
        * @param current_pose The current pose.
        * @param desired_state The desired pose, linear velocity, and angular velocity from a trajectory.
        * @return The next controller output.
        """
        return self.calculate_from_poses(
            current_pose,
            desired_state.pose,
            desired_state.velocity,
            desired_state.velocity * desired_state.curvature
        )
