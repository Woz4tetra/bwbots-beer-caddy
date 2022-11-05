# Adapted from edu.wpi.first.math.controller.HolonomicDriveController
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.

import rospy
import math
from typing import Optional
from ..robot_state import Pose2d, Velocity
from .pid import PIDController
from .profiled_pid import ProfiledPIDController
from .trapezoid_profile import State
from .controller import Controller


class NonHolonomicDriveController(Controller):

    def __init__(self, 
            x_controller: PIDController,
            y_controller: PIDController,
            theta_controller: ProfiledPIDController,
            turn_in_place_threshold: float = math.pi / 8.0) -> None:
        """
        * Constructs a holonomic drive controller.
        *
        * @param x_controller A PID Controller to respond to error in the field-relative x direction.
        * @param y_controller A PID Controller to respond to error in the field-relative y direction.
        * @param theta_controller A profiled PID controller to respond to error in angle.
        """
        super().__init__()

        self.x_controller = x_controller
        self.y_controller = y_controller
        self.theta_controller = theta_controller

        self.turn_in_place_threshold = turn_in_place_threshold

    def calculate(self, **kwargs) -> Velocity:
        current_pose: Pose2d = kwargs["current_pose"]
        pose_ref: Optional[Pose2d] = kwargs.get("pose_ref", None)
        linear_velocity_ref_meters: Optional[float] = kwargs.get("linear_velocity_ref_meters", None)
        angular_velocity_ref: Optional[float] = kwargs.get("angular_velocity_ref", None)
        desired_state: Optional[State] = kwargs.get("desired_state", None)
        linear_min_velocity: float = kwargs.get("linear_min_velocity", 0.0)
        theta_min_velocity: float = kwargs.get("theta_min_velocity", 0.0)
        allow_reverse: bool = kwargs.get("allow_reverse", False)

        if pose_ref is not None:
            return self.calculate_from_poses(
                current_pose,
                pose_ref,
                linear_velocity_ref_meters,
                angular_velocity_ref,
                linear_min_velocity,
                theta_min_velocity,
                allow_reverse)
        elif desired_state is not None:
            return self.calculate_from_state(
                current_pose,
                desired_state,
                linear_min_velocity,
                theta_min_velocity,
                allow_reverse
            )
        else:
            raise ValueError(f"Invalid parameters for {self.__class__.__name__}.calculate: {kwargs}")
    
    def calculate_from_poses(self, 
            current_pose: Pose2d,
            pose_ref: Pose2d,
            linear_velocity_ref_meters: Optional[float],
            angular_velocity_ref: Optional[float],
            linear_min_velocity: float,
            theta_min_velocity: float,
            allow_reverse: bool) -> Velocity:
        """
        * Returns the next output of the holonomic drive controller.
        *
        * @param current_pose The current pose.
        * @param pose_ref The desired pose.
        * @param linear_velocity_ref_meters The linear velocity reference.
        * @return The next output of the holonomic drive controller.
        """
        
        self.pose_error = pose_ref.relative_to(current_pose)
        rospy.loginfo(f"self.pose_error: {self.pose_error}")

        if self.pose_error.distance() > self.pose_tolerance.distance():
            self.pose_error.theta = self.pose_error.heading()
        
        is_reversed = False
        if allow_reverse and abs(self.pose_error.theta) > math.pi / 2.0:
            self.pose_error = self.pose_error.rotate_by(math.pi)
            is_reversed = True

        vx = self.x_controller.calculate(-self.pose_error.x, 0.0)
        vt = self.y_controller.calculate(-self.pose_error.y, 0.0)
        vt += self.theta_controller.calculate(-self.pose_error.theta, 0.0)
        if is_reversed:
            vx *= -1.0
            vt *= -1.0
        rospy.loginfo(f"vx: {vx}, vt: {vt}")

        if abs(vx) < 1E-6:
            vx = 0.0
        elif abs(vx) < linear_min_velocity:
            vx = math.copysign(linear_min_velocity, vx)

        if abs(vt) < 1E-6:
            vt = 0.0
        elif abs(vt) < theta_min_velocity:
            vt = math.copysign(theta_min_velocity, vt)

        if linear_velocity_ref_meters is not None:
            vx = PIDController.clamp(vx, -linear_velocity_ref_meters, linear_velocity_ref_meters)
        if angular_velocity_ref is not None:
            vt = PIDController.clamp(vt, -angular_velocity_ref, angular_velocity_ref)

        if abs(self.pose_error.theta) > self.turn_in_place_threshold:
            return Velocity(0.0, 0.0, vt)
        else:
            return Velocity(vx, 0.0, vt)

    def calculate_from_state(self,
            current_pose: Pose2d,
            desired_state: State,
            linear_min_velocity: float,
            theta_min_velocity: float,
            allow_reverse: bool) -> Velocity:
        """
        * Returns the next output of the holonomic drive controller.
        *
        * @param currentPose The current pose.
        * @param desiredState The desired trajectory state.
        * @param angleRef The desired end-angle.
        * @return The next output of the holonomic drive controller.
        """
        return self.calculate(
            current_pose,
            desired_state.pose_meters,
            desired_state.velocity_meters_per_second,
            linear_min_velocity,
            theta_min_velocity,
            allow_reverse
        )
