#!/usr/bin/env python3
import actionlib
import rospy

from bw_dispensers.dispenser.mqtt_dispenser import MqttDispense
from bw_dispensers.dispenser.simulated_dispenser import SimulatedDispense
from bw_interfaces.msg import DispenseAction, DispenseGoal, DispenseResult
from bw_tools.typing.basic import get_param


class BwDispensers:
    def __init__(self) -> None:
        node_name = "bw_dispensers"
        rospy.init_node(node_name)
        rospy.on_shutdown(self.shutdown_hook)

        self.timeout = rospy.Duration(get_param("~timeout", 5.0))
        self.simulated = get_param("~simulated", False)
        self.mqtt_server = get_param("~mqtt_server", "0.0.0.0")

        if self.simulated:
            self.dispense_client = SimulatedDispense()
        else:
            self.dispense_client = MqttDispense(self.mqtt_server)

        self.action_server = actionlib.SimpleActionServer(
            "dispense", DispenseAction, execute_cb=self.action_callback, auto_start=False
        )
        self.action_server.start()
        rospy.loginfo(f"{node_name} is ready")

    def action_callback(self, goal: DispenseGoal):
        dispenser_name = goal.dispenser_name
        timeout = goal.timeout

        result = DispenseResult()

        start_time = rospy.Time.now()
        current_time = rospy.Time.now()
        aborted = False
        if timeout.to_sec() <= 0.0:
            timeout = self.timeout
            rospy.loginfo(f"Using default timeout: {timeout}")

        self.dispense_client.start_dispense(dispenser_name)

        while current_time - start_time < timeout:
            current_time = rospy.Time.now()
            if self.action_server.is_preempt_requested():
                aborted = True
                break

            if self.dispense_client.is_done_dispensing():
                result.success = True
                rospy.loginfo("Dispense completed successfully.")
                break

        if aborted:
            self.action_server.set_aborted(result, "Interrupted while dispensing")
        else:
            if result.success:
                rospy.loginfo(f"Dispensed successfully from {dispenser_name}")
            else:
                rospy.loginfo(f"Failed to dispense from {dispenser_name}")
            self.action_server.set_succeeded(result)

    def run(self):
        rospy.spin()

    def shutdown_hook(self):
        self.dispense_client.close()


def main():
    node = BwDispensers()
    node.run()


if __name__ == "__main__":
    main()
