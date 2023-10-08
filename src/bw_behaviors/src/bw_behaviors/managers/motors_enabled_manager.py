from typing import Optional

import rospy
from std_msgs.msg import Bool

from bw_tools.structs.bool_stamped import BoolStamped


class MotorsEnabledManager:
    def __init__(self) -> None:
        self.set_enabled_pub = rospy.Publisher("set_motors_enabled", Bool, queue_size=10)
        self.is_enabled_sub = rospy.Subscriber(
            "are_motors_enabled", Bool, self.are_motors_enabled_callback, queue_size=10
        )
        self.are_motors_enabled = BoolStamped.auto(False)
        self.motors_enabled_command = BoolStamped.auto(False)
        self.wakeup_time = rospy.Time.now()

    def set_enable(self, motors_enabled: bool):
        self.motors_enabled_command = BoolStamped.auto(motors_enabled)
        self.set_enabled_pub.publish(Bool(motors_enabled))

    def are_motors_enabled_callback(self, msg: Bool) -> None:
        self.are_motors_enabled = BoolStamped.auto(msg.data)

    def is_enabled(self) -> Optional[bool]:
        if self.are_motors_enabled.stamp > self.wakeup_time.to_sec():
            return self.are_motors_enabled.state
        else:
            return None
