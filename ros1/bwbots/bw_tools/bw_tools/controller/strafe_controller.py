import rospy
from enum import Enum, auto
import math
from typing import Optional
from ..robot_state import Pose2d, Velocity
from .pid import PIDController
from .controller import Controller

class StrafeControllerState(Enum):
    START = auto()
    INITIAL_TURN = auto()
    STRAFING = auto()
    END_TURN = auto()

class StrafeController(Controller):
    def __init__(self, x_controller: PIDController, y_controller: PIDController, theta_controller: PIDController, strafe_angle_limit: float) -> None:
        """
        * Constructs a strafe controller.
        *
        * @param x_controller A PID Controller to respond to error in the field-relative x direction.
        * @param y_controller A PID Controller to respond to error in the field-relative y direction.
        * @param theta_controller A profiled PID controller to respond to error in angle.
        """
        super().__init__()

        self.x_controller = x_controller
        self.y_controller = y_controller
        self.theta_controller = theta_controller
        self.strafe_angle_limit = strafe_angle_limit
        self.state = StrafeControllerState.START

    def reset(self):
        self.state = StrafeControllerState.START

    def calculate(self, **kwargs) -> Velocity:
        current_pose: Pose2d = kwargs["current_pose"]
        pose_ref: Optional[Pose2d] = kwargs.get("pose_ref", None)
        linear_velocity_ref: Optional[float] = kwargs.get("linear_velocity_ref", None)
        angular_velocity_ref: Optional[float] = kwargs.get("angular_velocity_ref", None)
        linear_min_velocity: float = kwargs.get("linear_min_velocity", 0.0)
        linear_zero_velocity: float = kwargs.get("linear_zero_velocity", 1E-6)
        theta_min_velocity: float = kwargs.get("theta_min_velocity", 0.0)
        theta_zero_velocity: float = kwargs.get("theta_zero_velocity", 1E-6)
        allow_reverse: bool = kwargs.get("allow_reverse", False)

        pose_error = pose_ref.relative_to(current_pose)
        self.pose_error = Pose2d.from_state(pose_error)

        is_reversed = False
        if allow_reverse and abs(self.pose_error.heading()) > math.pi / 2.0:
            new_pose_error = pose_error.rotate_by(math.pi)
            pose_error.x = new_pose_error.x
            pose_error.y = new_pose_error.y
            is_reversed = True

        if self.state == StrafeControllerState.START:
            self.state = StrafeControllerState.INITIAL_TURN
        
        if self.state == StrafeControllerState.INITIAL_TURN:
            vel = Velocity(theta=self.theta_controller.calculate(0.0, pose_error.theta))
            if abs(self.pose_error.theta) < self.pose_tolerance.theta:
                self.state = StrafeControllerState.STRAFING
        elif self.state == StrafeControllerState.STRAFING:
            vel = Velocity(
                x=self.x_controller.calculate(0.0, pose_error.x),
                y=self.y_controller.calculate(0.0, pose_error.y)
            )
            if abs(self.pose_error.x) < self.pose_tolerance.x and \
                    abs(self.pose_error.y) < self.pose_tolerance.y:
                self.state = StrafeControllerState.END_TURN
        elif self.state == StrafeControllerState.END_TURN:
            vel = Velocity(theta=self.theta_controller.calculate(0.0, pose_error.theta))

        vel = Velocity(
            PIDController.clamp_region(vel.x, linear_min_velocity, linear_velocity_ref, linear_zero_velocity),
            PIDController.clamp_region(vel.y, linear_min_velocity, linear_velocity_ref, linear_zero_velocity),
            PIDController.clamp_region(vel.theta, theta_min_velocity, angular_velocity_ref, theta_zero_velocity)
        )

        if is_reversed:
            vel = -vel

        return vel
