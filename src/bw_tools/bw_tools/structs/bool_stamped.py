from dataclasses import dataclass

import rospy
from std_msgs.msg import Bool


@dataclass
class BoolStamped:
    stamp: float
    state: bool

    @classmethod
    def auto(cls, state: bool, stamp: float = float("nan")):
        if stamp != stamp:
            stamp = rospy.Time.now().to_sec()
        return cls(stamp, state)

    def to_msg(self) -> Bool:
        return Bool(self.state)
