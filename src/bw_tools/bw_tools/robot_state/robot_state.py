import math
from typing import List, cast
import numpy as np
import tf_conversions
from geometry_msgs.msg import Quaternion, Pose, Twist


class State:
    def __init__(self, x=0.0, y=0.0, theta=0.0):
        self.x = x  # meters
        self.y = y  # meters
        self.theta = theta  # radians

    @classmethod
    def from_xyt(cls, x=0.0, y=0.0, theta=0.0):
        self = cls()
        self.x = x
        self.y = y
        self.theta = theta
        return self

    @classmethod
    def from_state(cls, state):
        self = cls()
        self.x = state.x
        self.y = state.y
        self.theta = state.theta
        return self

    @classmethod
    def from_ros_pose(cls, pose):
        self = cls()
        self.x = pose.position.x
        self.y = pose.position.y
        self.theta = cls.normalize_theta(cls.theta_from_quat(pose.orientation))
        return self

    @staticmethod
    def theta_from_quat(quaternion):
        return tf_conversions.transformations.euler_from_quaternion(
            (quaternion.x, quaternion.y, quaternion.z, quaternion.w)
        )[2]

    def get_theta_as_quat(self) -> Quaternion:
        quat = self.get_theta_as_quat_as_list()

        quat_msg = Quaternion()
        quat_msg.x = quat[0]
        quat_msg.y = quat[1]
        quat_msg.z = quat[2]
        quat_msg.w = quat[3]
        return quat_msg

    def get_theta_as_quat_as_list(self) -> List[float]:
        return tf_conversions.transformations.quaternion_from_euler(
            0.0, 0.0, self.theta
        ).tolist()

    def transform_by(self, other):
        if not isinstance(other, self.__class__):
            raise ValueError(
                "Can't transform %s to %s" % (self.__class__, other.__class__)
            )
        state = self.__class__.from_state(self)
        state = state.rotate_by(other.theta)
        state.x += other.x
        state.y += other.y
        state.theta = other.theta + self.theta

        state.theta = self.normalize_theta(state.theta)
        return state

    def relative_to(self, other):
        if not isinstance(other, self.__class__):
            raise ValueError("%s is not of type %s" % (repr(other), self.__class__))
        state = self.__class__.from_state(self)
        state.x -= other.x
        state.y -= other.y
        state = state.rotate_by(-other.theta)
        state.theta = self.normalize_theta(state.theta)
        return state

    def rotate_by(self, theta):
        """
        Apply rotation matrix (defined by theta)
        """
        state = self.__class__()
        state.x = self.x * math.cos(theta) - self.y * math.sin(theta)
        state.y = self.x * math.sin(theta) + self.y * math.cos(theta)
        state.theta = self.theta + theta
        return state

    def delta(self, other, states="xy"):
        if not isinstance(other, self.__class__):
            raise ValueError(
                "Can't subtract %s and %s" % (self.__class__, other.__class__)
            )

        state = self.__class__.from_state(self)
        for key in states:
            if key == "x":
                state.x = self.x - other.x
            elif key == "y":
                state.y = self.y - other.y
            elif key == "t":
                state.theta = self.theta - other.theta

        return state

    def add(self, other, states="xy"):
        if not isinstance(other, self.__class__):
            raise ValueError(
                "Can't subtract %s and %s" % (self.__class__, other.__class__)
            )

        state = self.__class__.from_state(self)
        for key in states:
            if key == "x":
                state.x = self.x + other.x
            elif key == "y":
                state.y = self.y + other.y
            elif key == "t":
                state.theta = self.theta + other.theta

        return state

    @staticmethod
    def _clip(x, lower, upper):
        if x == 0.0:
            return x
        if lower is None and upper is None:
            return x
        elif lower is None and upper is not None:
            clipped_x = min(abs(x), upper)
        if lower is not None and upper is None:
            clipped_x = max(abs(x), lower)
        else:
            clipped_x = min(max(abs(x), lower), upper)
        clipped_x = math.copysign(clipped_x, x)
        return clipped_x

    def clip(self, lower, upper):
        state = self.__class__.from_state(self)
        if self.magnitude() < lower.magnitude():
            state.theta = State._clip(self.theta, lower.theta, upper.theta)
        if abs(self.theta) < lower.theta:
            state.x = State._clip(self.x, lower.x, upper.x)
            state.y = State._clip(self.y, lower.y, upper.y)
        return state

    def magnitude(self):
        return math.sqrt(self.x * self.x + self.y * self.y)

    def distance(self, other=None):
        if (
            other is None
        ):  # if other is None, assume you're getting distance from the origin
            other = self.__class__()
        if not isinstance(other, self.__class__):
            raise ValueError(
                "Can't get distance from %s to %s" % (self.__class__, other.__class__)
            )
        dx = self.x - other.x
        dy = self.y - other.y
        return math.sqrt(dx * dx + dy * dy)

    def heading(self, other=None):
        if other is None:
            return math.atan2(self.y, self.x)
        else:
            if not isinstance(other, self.__class__):
                raise ValueError(
                    "Can't get heading from %s to %s"
                    % (self.__class__, other.__class__)
                )
            dx = self.x - other.x
            dy = self.y - other.y
            return math.atan2(dy, dx)

    @classmethod
    def median(cls, poses: List["Pose2d"]):
        self = cls()
        xs = [pose.x for pose in poses]
        ys = [pose.y for pose in poses]
        thetas = [pose.theta for pose in poses]
        self.x = np.median(xs)
        self.y = np.median(ys)
        self.theta = np.median(thetas)
        return self

    @classmethod
    def average(cls, poses: List["Pose2d"]):
        self = cls()
        xs = [pose.x for pose in poses]
        ys = [pose.y for pose in poses]
        thetas = [pose.theta for pose in poses]
        self.x = np.average(xs)
        self.y = np.average(ys)
        self.theta = np.average(thetas)
        return self

    @classmethod
    def stddev(cls, poses: List["Pose2d"]):
        self = cls()
        xs = [pose.x for pose in poses]
        ys = [pose.y for pose in poses]
        thetas = [pose.theta for pose in poses]
        self.x = np.std(xs)
        self.y = np.std(ys)
        self.theta = np.std(thetas)
        return self

    @classmethod
    def normalize_theta(cls, theta):
        # normalize theta to -pi..pi
        theta = math.fmod(theta, 2 * math.pi)
        if abs(theta) > math.pi:
            if theta > 0:
                return theta - 2 * math.pi
            else:
                return theta + 2 * math.pi
        return theta

    def get_normalize_theta(self):
        return self.normalize_theta(self.theta)

    def to_list(self, states="xyt"):
        output = []
        for state in states:
            if state == "x":
                output.append(float(self.x))
            elif state == "y":
                output.append(float(self.y))
            elif state == "t":
                output.append(float(self.theta))
        return output

    @classmethod
    def to_array(cls, poses: list):
        return np.array([pose.to_list() for pose in poses])

    def to_ros_pose(self):
        ros_pose = Pose()
        ros_pose.position.x = self.x
        ros_pose.position.y = self.y
        ros_pose.orientation = self.get_theta_as_quat()
        return ros_pose

    def __pos__(self):
        return self

    def __neg__(self):
        new_state = self.from_state(self)
        new_state.x = -new_state.x
        new_state.y = -new_state.y
        new_state.theta = -new_state.theta
        return new_state

    def __add__(self, other):
        if not isinstance(other, self.__class__):
            raise ValueError("Can't add %s and %s" % (self.__class__, other.__class__))

        state = self.__class__()
        state.x = self.x + other.x
        state.y = self.y + other.y
        state.theta = self.theta + other.theta
        return state

    def __sub__(self, other):
        if not isinstance(other, self.__class__):
            raise ValueError(
                "Can't subtract %s and %s" % (self.__class__, other.__class__)
            )

        state = self.__class__()
        state.x = self.x - other.x
        state.y = self.y - other.y
        state.theta = self.theta - other.theta
        return state

    def __mul__(self, other):
        state = self.__class__()
        if isinstance(other, self.__class__):
            state.x = self.x * other.x
            state.y = self.y * other.y
            state.theta = self.theta * other.theta
        elif isinstance(other, int) or isinstance(other, float):
            state.x = self.x * other
            state.y = self.y * other
            state.theta = self.theta * other
        else:
            raise ValueError(
                "Can't multiply %s and %s" % (self.__class__, other.__class__)
            )
        return state

    def __truediv__(self, other):
        state = self.__class__()
        if isinstance(other, self.__class__):
            state.x = self.x / other.x
            state.y = self.y / other.y
            state.theta = self.theta / other.theta
        elif isinstance(other, int) or isinstance(other, float):
            state.x = self.x / other
            state.y = self.y / other
            state.theta = self.theta / other
        else:
            raise ValueError(
                "Can't divide %s and %s" % (self.__class__, other.__class__)
            )
        return state

    def __abs__(self):
        return self.__class__.from_xyt(abs(self.x), abs(self.y), abs(self.theta))

    def __lt__(self, other):
        return self.x < other.x and self.y < other.y and self.theta < other.theta

    def __eq__(self, other):
        return self.x == other.x and self.y == other.y and self.theta == other.theta

    def __str__(self):
        return "%s(x=%0.4f, y=%0.4f, theta=%0.4f)" % (
            self.__class__.__name__,
            self.x,
            self.y,
            self.theta,
        )

    __repr__ = __str__


class Pose2d(State):
    def project(self, velocity: "Velocity", time_delta: float) -> "Pose2d":
        dx = velocity.x * time_delta
        dy = velocity.y * time_delta
        dtheta = velocity.theta * time_delta

        if abs(dtheta) < 1e-9:
            s = 1.0 - 1.0 / 6.0 * dtheta * dtheta
            c = 0.5 * dtheta
        else:
            s = math.sin(dtheta) / dtheta
            c = (1.0 - math.cos(dtheta)) / dtheta

        tx = dx * s - dy * c
        ty = dx * c + dy * s

        forwarded = cast(Pose2d, self.transform_by(Pose2d(tx, ty, dtheta)))
        return forwarded


class Velocity(State):
    @classmethod
    def from_poses(cls, old_pose: Pose2d, new_pose: Pose2d, dt: float) -> "Velocity":
        relative_pose = new_pose.relative_to(old_pose)
        velocity_pose = relative_pose / dt
        return cls.from_state(velocity_pose)

    @classmethod
    def from_global_relative_speeds(
        cls, vx: float, vy: float, vt: float, angle_ref: float
    ) -> "Velocity":
        self = cls(vx, vy, 0.0)
        self.rotate_by(-angle_ref)
        self.theta = vt
        return self

    @classmethod
    def from_ros_twist(cls, twist: Twist):
        self = cls()
        self.x = twist.linear.x
        self.y = twist.linear.y
        self.theta = twist.angular.z
        return self

    def to_ros_twist(self) -> Twist:
        ros_twist = Twist()
        ros_twist.linear.x = self.x
        ros_twist.linear.y = self.y
        ros_twist.angular.z = self.theta
        return ros_twist
