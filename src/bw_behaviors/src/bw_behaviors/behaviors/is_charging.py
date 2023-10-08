import rospy
from py_trees.behaviour import Behaviour
from py_trees.common import Status

from bw_behaviors.container import Container


class IsCharging(Behaviour):
    def __init__(self, container: Container, voltage_threshold: float, current_threshold: float):
        super().__init__(self.__class__.__name__)
        self.charge_manager = container.charge_manager
        self.voltage_threshold = voltage_threshold
        self.current_threshold = current_threshold

    def initialise(self) -> None:
        pass

    def update(self) -> Status:
        # TODO: if battery voltage is above threshold, the charger will not supply current even if contact is made
        # Use LIDAR to detect if we are charging
        battery_voltage = self.charge_manager.charge.battery_voltage
        charge_current = self.charge_manager.charge.charge_current
        rospy.loginfo(f"Charge state: {battery_voltage} V. {charge_current} A")
        if battery_voltage > self.voltage_threshold or charge_current > self.current_threshold:
            return Status.SUCCESS
        else:
            return Status.FAILURE
