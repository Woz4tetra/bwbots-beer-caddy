from threading import Lock

import rospy

from bw_interfaces.msg import ChargeState


class ChargeManager:
    def __init__(self) -> None:
        self.charge_sub = rospy.Subscriber("charger", ChargeState, self.charge_callback, queue_size=10)
        self._charge = ChargeState()
        self.lock = Lock()

    @property
    def charge(self) -> ChargeState:
        with self.lock:
            return self._charge

    def charge_callback(self, msg: ChargeState) -> None:
        with self.lock:
            self._charge = msg
