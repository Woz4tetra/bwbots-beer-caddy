#!/usr/bin/env python3
import rospy
import actionlib
from std_msgs.msg import String
from bw_interfaces.msg import DispenseAction, DispenseGoal, DispenseFeedback, DispenseResult


class BwDispensers:
    def __init__(self) -> None:
        node_name = "bw_dispensers"
        rospy.init_node(
            node_name,
            # disable_signals=True,
            # log_level=rospy.DEBUG
        )
        self.timeout = rospy.Duration(rospy.get_param("~timeout", 5.0))
        self.simulated = rospy.Duration(rospy.get_param("~simulated", False))
        
        if self.simulated:
            self.simulated_dispense_pub = rospy.Publisher("simulated_dispense", String, queue_size=5)
        else:
            self.simulated_dispense_pub = None

        self.action_server = actionlib.SimpleActionServer(
            "dispense",
            DispenseAction,
            execute_cb=self.action_callback, 
            auto_start=False
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
        while current_time - start_time < timeout:
            current_time = rospy.Time.now()
            if self.action_server.is_preempt_requested():
                aborted = True
                break

            if self.simulated:
                if self.simulated_dispense_pub.get_num_connections() > 0:
                    self.simulated_dispense_pub.publish(String(dispenser_name))
                    result.success = True
                    break
            else:
                result.success = True
                break
                # TODO: connect to dispenser over wifi/bluetooth
                # firmware contains name. Look up by name and send specialized dispense command

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
        
        
def main():
    node = BwDispensers()
    node.run()


if __name__ == "__main__":
    main()
