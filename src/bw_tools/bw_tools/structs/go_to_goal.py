from dataclasses import dataclass

from bw_interfaces.msg import GoToPoseGoal as RosGoToPoseGoal
from bw_tools.robot_state.robot_state import Pose2d, Pose2dStamped
from bw_tools.structs.header import Header
from bw_tools.typing.basic import seconds_to_duration


@dataclass
class GoToPoseGoal:
    goal: Pose2dStamped
    xy_tolerance: float
    yaw_tolerance: float
    timeout: float
    reference_linear_speed: float
    reference_angular_speed: float
    allow_reverse: bool
    rotate_in_place_start: bool
    rotate_while_driving: bool
    rotate_in_place_end: bool
    linear_max_accel: float
    linear_min_vel: float
    theta_max_accel: float
    theta_min_vel: float

    @classmethod
    def from_xyt(cls, frame_id: str, x: float, y: float, theta: float) -> "GoToPoseGoal":
        return cls(
            goal=Pose2dStamped(Header.auto(frame_id), Pose2d(x, y, theta)),
            xy_tolerance=0.025,
            yaw_tolerance=0.02,
            timeout=30.0,
            reference_linear_speed=0.75,
            reference_angular_speed=2.0,
            allow_reverse=False,
            rotate_in_place_start=True,
            rotate_while_driving=False,
            rotate_in_place_end=True,
            linear_max_accel=2.0,
            linear_min_vel=0.15,
            theta_max_accel=1.0,
            theta_min_vel=0.015,
        )

    @classmethod
    def from_msg(cls, msg: RosGoToPoseGoal) -> "GoToPoseGoal":
        return cls(
            goal=Pose2dStamped.from_msg(msg.goal),
            xy_tolerance=msg.xy_tolerance,
            yaw_tolerance=msg.yaw_tolerance,
            timeout=msg.timeout.to_sec(),
            reference_linear_speed=msg.reference_linear_speed,
            reference_angular_speed=msg.reference_angular_speed,
            allow_reverse=msg.allow_reverse,
            rotate_in_place_start=msg.rotate_in_place_start,
            rotate_while_driving=msg.rotate_while_driving,
            rotate_in_place_end=msg.rotate_in_place_end,
            linear_max_accel=msg.linear_max_accel,
            linear_min_vel=msg.linear_min_vel,
            theta_max_accel=msg.theta_max_accel,
            theta_min_vel=msg.theta_min_vel,
        )

    def to_msg(self) -> RosGoToPoseGoal:
        return RosGoToPoseGoal(
            goal=self.goal.to_msg(),
            xy_tolerance=self.xy_tolerance,
            yaw_tolerance=self.yaw_tolerance,
            timeout=seconds_to_duration(self.timeout),
            reference_linear_speed=self.reference_linear_speed,
            reference_angular_speed=self.reference_angular_speed,
            allow_reverse=self.allow_reverse,
            rotate_in_place_start=self.rotate_in_place_start,
            rotate_while_driving=self.rotate_while_driving,
            rotate_in_place_end=self.rotate_in_place_end,
            linear_max_accel=self.linear_max_accel,
            linear_min_vel=self.linear_min_vel,
            theta_max_accel=self.theta_max_accel,
            theta_min_vel=self.theta_min_vel,
        )
