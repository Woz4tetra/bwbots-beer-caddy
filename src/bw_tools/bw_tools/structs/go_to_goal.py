from dataclasses import dataclass

from bw_interfaces.msg import GoToPoseGoal as RosGoToPoseGoal
from bw_tools.robot_state.robot_state import Pose2d, Pose2dStamped
from bw_tools.structs.header import Header
from bw_tools.typing.basic import seconds_to_duration


@dataclass(frozen=True, eq=True)
class GoToPoseGoal:
    goal: Pose2dStamped
    linear_tolerance: float
    angle_tolerance: float
    timeout: float
    reference_linear_speed: float
    reference_angular_speed: float
    linear_max_accel: float
    linear_min_vel: float
    angle_max_accel: float
    angle_min_vel: float

    @classmethod
    def from_xyt(cls, frame_id: str, x: float, y: float, theta: float) -> "GoToPoseGoal":
        return cls(
            goal=Pose2dStamped(Header.auto(frame_id), Pose2d(x, y, theta)),
            linear_tolerance=0.025,
            angle_tolerance=0.02,
            timeout=30.0,
            reference_linear_speed=0.75,
            reference_angular_speed=2.0,
            linear_max_accel=2.0,
            linear_min_vel=0.15,
            angle_max_accel=1.0,
            angle_min_vel=0.015,
        )

    @classmethod
    def from_msg(cls, msg: RosGoToPoseGoal) -> "GoToPoseGoal":
        return cls(
            goal=Pose2dStamped.from_msg(msg.goal),
            linear_tolerance=msg.xy_tolerance,
            angle_tolerance=msg.yaw_tolerance,
            timeout=msg.timeout.to_sec(),
            reference_linear_speed=msg.reference_linear_speed,
            reference_angular_speed=msg.reference_angular_speed,
            linear_max_accel=msg.linear_max_accel,
            linear_min_vel=msg.linear_min_vel,
            angle_max_accel=msg.theta_max_accel,
            angle_min_vel=msg.theta_min_vel,
        )

    def to_msg(self) -> RosGoToPoseGoal:
        return RosGoToPoseGoal(
            goal=self.goal.to_msg(),
            linear_tolerance=self.linear_tolerance,
            angle_tolerance=self.angle_tolerance,
            timeout=seconds_to_duration(self.timeout),
            reference_linear_speed=self.reference_linear_speed,
            reference_angular_speed=self.reference_angular_speed,
            linear_max_accel=self.linear_max_accel,
            linear_min_vel=self.linear_min_vel,
            angle_max_accel=self.angle_max_accel,
            angle_min_vel=self.angle_min_vel,
        )
