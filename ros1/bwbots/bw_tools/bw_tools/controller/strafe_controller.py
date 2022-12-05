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
    def __init__(self, x_controller: PIDController, y_controller: Optional[PIDController], theta_controller: PIDController, strafe_angle_limit: float) -> None:
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
        assert self.x_controller is not None
        assert self.theta_controller is not None

    def reset(self):
        self.state = StrafeControllerState.START

    def calculate(self, **kwargs) -> Velocity:
        current_pose: Pose2d = kwargs["current_pose"]
        pose_ref: Optional[Pose2d] = kwargs.get("pose_ref", None)
        linear_velocity_ref: Optional[float] = kwargs.get("linear_velocity_ref", None)
        angular_velocity_ref: Optional[float] = kwargs.get("angular_velocity_ref", None)
        linear_x_min_velocity: float = kwargs.get("linear_x_min_velocity", 0.0)
        linear_y_min_velocity: float = kwargs.get("linear_y_min_velocity", 0.0)
        linear_zero_velocity: float = kwargs.get("linear_zero_velocity", 1E-6)
        theta_min_velocity: float = kwargs.get("theta_min_velocity", 0.0)
        theta_zero_velocity: float = kwargs.get("theta_zero_velocity", 1E-6)
        allow_reverse: bool = kwargs.get("allow_reverse", False)
        rotate_in_place_start: bool = kwargs.get("rotate_in_place_start", True)
        rotate_while_driving: bool = kwargs.get("rotate_while_driving", True)
        rotate_in_place_end: bool = kwargs.get("rotate_in_place_end", True)

        pose_error = pose_ref.relative_to(current_pose)
        self.pose_error = Pose2d.from_state(pose_error)

        is_reversed = False
        heading = self.pose_error.heading()
        if allow_reverse and abs(heading) > math.pi / 2.0:
            new_pose_error = pose_error.rotate_by(math.pi)
            pose_error.x = new_pose_error.x
            pose_error.y = new_pose_error.y
            is_reversed = True

        if self.state == StrafeControllerState.START:
            self.state = StrafeControllerState.INITIAL_TURN
        
        if self.state == StrafeControllerState.INITIAL_TURN:
            if rotate_in_place_start:
                vel = Velocity(theta=self.theta_controller.calculate(0.0, heading))
                if abs(heading) < self.pose_tolerance.theta:
                    self.state = StrafeControllerState.STRAFING
            else:
                self.state = StrafeControllerState.STRAFING
                vel = Velocity()
        elif self.state == StrafeControllerState.STRAFING:
            if rotate_while_driving:
                if rotate_in_place_end:
                    vt = self.theta_controller.calculate(0.0, pose_error.theta)
                else:
                    vt = self.theta_controller.calculate(0.0, heading)
            else:
                vt = 0.0
            vel = Velocity(
                x=self.x_controller.calculate(0.0, pose_error.x),
                y=0.0 if self.y_controller is None else self.y_controller.calculate(0.0, pose_error.y),
                theta=vt
            )
            if rotate_while_driving:
                if self.at_reference():
                    self.state = StrafeControllerState.END_TURN
            else:
                if abs(self.pose_error.x) < self.pose_tolerance.x and \
                        abs(self.pose_error.y) < self.pose_tolerance.y:
                    self.state = StrafeControllerState.END_TURN

        elif self.state == StrafeControllerState.END_TURN:
            if rotate_in_place_end:
                if abs(self.pose_error.x) >= self.pose_tolerance.x or \
                    abs(self.pose_error.y) >= self.pose_tolerance.y:
                    self.state = StrafeControllerState.STRAFING
                vel = Velocity(x=0.0, y=0.0, theta=self.theta_controller.calculate(0.0, pose_error.theta))
            else:
                if not self.at_reference():
                    self.state = StrafeControllerState.STRAFING
                vel = Velocity()

        vel = Velocity(
            PIDController.clamp_region(vel.x, linear_x_min_velocity, linear_velocity_ref, linear_zero_velocity),
            PIDController.clamp_region(vel.y, linear_y_min_velocity, linear_velocity_ref, linear_zero_velocity),
            PIDController.clamp_region(vel.theta, theta_min_velocity, angular_velocity_ref, theta_zero_velocity)
        )

        if is_reversed:
            vel = -vel

        return vel
