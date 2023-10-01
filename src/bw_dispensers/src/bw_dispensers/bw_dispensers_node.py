#!/usr/bin/env python3
from collections import namedtuple
from typing import Optional

import actionlib
import paho.mqtt.client as mqtt
import rospy
from std_msgs.msg import String

from bw_interfaces.msg import DispenseAction, DispenseFeedback, DispenseGoal, DispenseResult


class BwDispensers:
    def __init__(self) -> None:
        node_name = "bw_dispensers"
        rospy.init_node(
            node_name,
            # disable_signals=True,
            # log_level=rospy.DEBUG
        )
        rospy.on_shutdown(self.shutdown_hook)
        self.timeout = rospy.Duration(rospy.get_param("~timeout", 5.0))
        self.simulated = rospy.get_param("~simulated", False)
        self.mqtt_server = rospy.get_param("~mqtt_server", "0.0.0.0")

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
        while current_time - start_time < timeout:
            current_time = rospy.Time.now()
            if self.action_server.is_preempt_requested():
                aborted = True
                break

            has_drink = self.dispense_client.has_drink()
            if has_drink is None:
                continue
            if not has_drink:
                result.success = False
                rospy.logwarn("Dispenser doesn't have any drinks!")
                break

            if not self.dispense_client.start_dispense(dispenser_name):
                result.success = False
                rospy.logwarn("Failed to send start dispensing command!")
                break

            is_done = self.dispense_client.is_done_dispensing()
            if is_done is None:
                continue
            if is_done:
                result.success = True
                rospy.loginfo("Dispense completed successfully.")
                break
            else:
                result.success = False
                rospy.loginfo("Dispense completed unsuccessfully.")
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


BooleanState = namedtuple("BooleanState", ["timestamp", "state"])


class DispenseClientBase:
    def __init__(self) -> None:
        pass

    def has_drink(self) -> Optional[bool]:
        return None

    def start_dispense(self, dispenser_name) -> bool:
        return False

    def is_done_dispensing(self) -> Optional[bool]:
        return False

    def close(self) -> None:
        pass


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


class MqttDispense(DispenseClientBase):
    def __init__(self, mqtt_server) -> None:
        # Give a name to this MQTT client
        self.client = mqtt.Client("drum_dispenser")
        self.client.message_callback_add("is_dispensing", self.on_is_dispensing)
        self.client.message_callback_add("dispense_done", self.on_dispense_done)
        self.client.message_callback_add("has_drink", self.on_has_drink)

        # IP address of your MQTT broker, using ipconfig to look up it
        self.client.connect(mqtt_server, 1883)

        self.client.loop_start()
        self.client.subscribe("is_dispensing/#")
        self.client.subscribe("dispense_done/#")
        self.client.subscribe("has_drink/#")

        self.state = {
            "is_dispensing": BooleanState(rospy.Time.now(), False),
            "dispense_done": BooleanState(rospy.Time.now(), False),
            "has_drink": BooleanState(rospy.Time.now(), False),
        }
        self.start_dispense_time = rospy.Time.now()

    def close(self) -> None:
        self.client.loop_stop()

    def has_drink(self) -> Optional[bool]:
        has_drink_state = self.state["has_drink"]
        if has_drink_state.timestamp > self.start_dispense_time:
            return has_drink_state.state
        else:
            return None

    def start_dispense(self, dispenser_name) -> bool:
        self.start_dispense_time = rospy.Time.now()
        return self.publish_start_dispense(dispenser_name)

    def is_done_dispensing(self) -> Optional[bool]:
        is_done_state = self.state["dispense_done"]
        if is_done_state.timestamp > self.start_dispense_time:
            return is_done_state.state
        else:
            return None

    def on_is_dispensing(self, client, userdata, msg):
        payload = str(msg.payload.decode("utf-8"))
        device_name, state = payload.split("\t")
        is_dispensing = state == "1"
        self.state["is_dispensing"] = BooleanState(rospy.Time.now(), is_dispensing)
        rospy.loginfo(f"{device_name} is {'' if is_dispensing else 'not '}dispensing")

    def on_has_drink(self, client, userdata, msg):
        payload = str(msg.payload.decode("utf-8"))
        device_name, state = payload.split("\t")
        has_drink = state == "1"
        self.state["has_drink"] = BooleanState(rospy.Time.now(), has_drink)
        rospy.loginfo(f"{device_name} does {'' if has_drink else 'not '}have a drink")

    def on_dispense_done(self, client, userdata, msg):
        payload = str(msg.payload.decode("utf-8"))
        device_name, state = payload.split("\t")
        success = state == "1"
        self.state["dispense_done"] = BooleanState(rospy.Time.now(), success)
        rospy.loginfo(f"{device_name} finished dispensing {'' if success else 'un'}successfully")

    def publish_start_dispense(self, device_name: str):
        info = self.client.publish("start_dispense", device_name.encode("utf-8"), qos=0)
        info.wait_for_publish()
        if not info.is_published():
            rospy.loginfo("Dispense message failed to publish!")
            return False
        else:
            return True


def main():
    node = BwDispensers()
    node.run()


if __name__ == "__main__":
    main()
