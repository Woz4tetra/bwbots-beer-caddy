import rospy
from typing import cast, TypeVar


T = TypeVar("T")


def get_param(path: str, default: T) -> T:
    value = rospy.get_param(path, default)
    if value is None:
        return default
    else:
        return cast(T, value)


def seconds_to_duration(seconds: float) -> rospy.Duration:
    return rospy.Duration(float(seconds))  # type: ignore
