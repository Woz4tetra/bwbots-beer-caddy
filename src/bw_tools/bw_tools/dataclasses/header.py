from dataclasses import dataclass
from typing import Optional

import rospy
from std_msgs.msg import Header as RosHeader

from bw_tools.dataclasses.context_sequence_counter import ContextSequenceCounter
from bw_tools.typing.basic import seconds_to_duration


@dataclass
class Header:
    stamp: float
    frame_id: str
    seq: int

    @classmethod
    def auto(cls, frame_id: str = '', stamp: float = float("nan"), seq: Optional[int] = None) -> "Header":
        if stamp != stamp:
            stamp = rospy.Time.now().to_sec()
        if seq is None:
            seq = ContextSequenceCounter.seq()
        return cls(stamp, frame_id, seq)

    @classmethod
    def from_msg(cls, msg: RosHeader) -> "Header":
        return cls(msg.stamp.to_sec(), msg.frame_id, msg.seq)

    def to_msg(self) -> RosHeader:
        return RosHeader(stamp=seconds_to_duration(self.stamp), frame_id=self.frame_id, seq=self.seq)
