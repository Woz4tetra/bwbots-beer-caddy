import rospy
import math
from typing import Optional
from ..robot_state import Pose2d, Velocity
from .pid import PIDController
from .controller import Controller


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

    def calculate(self, **kwargs) -> Velocity:
        current_pose: Pose2d = kwargs["current_pose"]
        pose_ref: Optional[Pose2d] = kwargs.get("pose_ref", None)
        linear_velocity_ref: Optional[float] = kwargs.get("linear_velocity_ref", None)
        angular_velocity_ref: Optional[float] = kwargs.get("angular_velocity_ref", None)
        linear_min_velocity: float = kwargs.get("linear_min_velocity", 0.0)
        theta_min_velocity: float = kwargs.get("theta_min_velocity", 0.0)
        allow_reverse: bool = kwargs.get("allow_reverse", False)

        pose_error = self.pose_error = pose_ref.relative_to(current_pose)

        is_reversed = False
        if allow_reverse and abs(self.pose_error.heading()) > math.pi / 2.0:
            new_pose_error = pose_error.rotate_by(math.pi)
            pose_error.x = new_pose_error.x
            pose_error.y = new_pose_error.y
            is_reversed = True

        strafe_direction = abs(pose_error.heading()) % (math.pi / 2.0)
        if strafe_direction > self.strafe_angle_limit:
            rospy.logwarn(f"Goal pose is outside strafe range: {pose_error.heading()}. Strafe limit: {self.strafe_angle_limit}")
            return Velocity(0.0, 0.0, 0.0)

        vx = self.x_controller.calculate(0.0, pose_error.x)
        vy = self.y_controller.calculate(0.0, pose_error.y)
        vt = self.theta_controller.calculate(0.0, pose_error.theta)

        vx = PIDController.clamp_region(vx, linear_min_velocity, linear_velocity_ref)
        vy = PIDController.clamp_region(vy, linear_min_velocity, linear_velocity_ref)
        vt = PIDController.clamp_region(vt, theta_min_velocity, angular_velocity_ref)

        vel = Velocity(vx, vy, vt)

        if is_reversed:
            # vel.x = -vel.x
            # vel.y = -vel.y
            vel = -vel

        return vel
