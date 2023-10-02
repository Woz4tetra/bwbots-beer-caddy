import time
from dataclasses import dataclass
from typing import Optional

import paho.mqtt.client as mqtt
import rospy

from bw_dispensers.dispenser import DispenseClientBase


@dataclass
class BoolStamped:
    stamp: rospy.Time
    state: bool


class MqttDispense(DispenseClientBase):
    def __init__(self, mqtt_server) -> None:
        self.dispense_speed = 255
        self.post_delay = 100

        # Give a name to this MQTT client
        self.client = mqtt.Client("drum_dispenser")
        self.client.message_callback_add("is_dispensing", self.on_is_dispensing)

        # IP address of your MQTT broker, using ipconfig to look up it
        self.client.connect(mqtt_server, 1883)

        self.client.loop_start()
        self.client.subscribe("is_dispensing/#")

        self.is_dispensing = BoolStamped(rospy.Time.now(), False)
        self.is_done = BoolStamped(rospy.Time.now(), False)
        self.start_dispense_time = rospy.Time.now()

    def close(self) -> None:
        self.client.loop_stop()

    def start_dispense(self, dispenser_name):
        self.is_done = BoolStamped(rospy.Time.now(), False)
        self.start_dispense_time = rospy.Time.now()
        self.publish_set_speed(self.dispense_speed, self.post_delay)
        time.sleep(0.25)
        self.publish_start_dispense(dispenser_name)

    def is_done_dispensing(self) -> Optional[bool]:
        if self.is_done.stamp > self.start_dispense_time:
            return self.is_done.state
        else:
            return None

    def on_is_dispensing(self, client, userdata, msg):
        payload = str(msg.payload.decode("utf-8"))
        device_name, state = payload.split("\t")
        is_dispensing_state = state == "1"
        if self.is_dispensing.state and not is_dispensing_state:
            self.is_done = BoolStamped(rospy.Time.now(), True)
        self.is_dispensing = BoolStamped(rospy.Time.now(), is_dispensing_state)
        rospy.loginfo(f"{device_name} is {'' if is_dispensing_state else 'not '}dispensing")

    def publish_start_dispense(self, device_name: str):
        self.client.publish("start_dispense", device_name.encode("utf-8"), qos=0)

    def publish_set_speed(self, speed: int, post_delay: int):
        data = speed, post_delay
        packet = ",".join([str(x) for x in data])
        self.client.publish("dispense_speed", str(packet).encode("utf-8"), qos=0)
