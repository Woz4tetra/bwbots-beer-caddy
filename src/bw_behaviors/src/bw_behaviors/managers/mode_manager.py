import rospy

from bw_behaviors.modes import Mode
from bw_interfaces.msg import BehaviorMode


class ModeManager:
    def __init__(self) -> None:
        self.mode_sub = rospy.Subscriber("behavior_mode", BehaviorMode, self.mode_callback, queue_size=10)
        self.mode = Mode.IDLE

    def set_mode(self, mode: Mode) -> None:
        self.mode = mode
        rospy.loginfo(f"Setting mode to {self.mode}")

    def mode_callback(self, msg: BehaviorMode) -> None:
        self.set_mode(Mode(msg.mode))
