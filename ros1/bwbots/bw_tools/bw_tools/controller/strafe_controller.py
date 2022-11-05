import math
from typing import Optional
from ..robot_state import Pose2d, Velocity
from .pid import PIDController
from .profiled_pid import ProfiledPIDController
from .trapezoid_profile import State
from .controller import Controller


class StrafeController(Controller):
    def __init__(self, x_controller: PIDController, y_controller: PIDController, theta_controller: ProfiledPIDController, strafe_angle_limit: float) -> None:
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

    def calculate(self, **kwargs) -> Velocity:
        current_pose: Pose2d = kwargs["current_pose"]
        pose_ref: Optional[Pose2d] = kwargs.get("pose_ref", None)
        linear_velocity_ref: Optional[float] = kwargs.get("linear_velocity_ref", None)
        angular_velocity_ref: Optional[float] = kwargs.get("angular_velocity_ref", None)
        desired_state: Optional[State] = kwargs.get("desired_state", None)
        linear_min_velocity: float = kwargs.get("linear_min_velocity", 0.0)
        theta_min_velocity: float = kwargs.get("theta_min_velocity", 0.0)
        allow_reverse: bool = kwargs.get("allow_reverse", False)

        self.pose_error = pose_ref.relative_to(current_pose)
        

