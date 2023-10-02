from typing import Optional

import rospy
from std_msgs.msg import String

from bw_dispensers.dispenser import DispenseClientBase


class SimulatedDispense(DispenseClientBase):
    def __init__(self) -> None:
        self.simulated_dispense_pub = rospy.Publisher("simulated_dispense", String, queue_size=5)

    def has_drink(self) -> Optional[bool]:
        return True

    def start_dispense(self, dispenser_name) -> bool:
        if self.simulated_dispense_pub.get_num_connections() > 0:
            self.simulated_dispense_pub.publish(String(dispenser_name))
            return True
        else:
            return False

    def is_done_dispensing(self) -> Optional[bool]:
        return True

    def close(self) -> None:
        pass
