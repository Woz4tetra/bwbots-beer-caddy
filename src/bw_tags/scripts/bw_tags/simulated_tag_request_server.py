#!/usr/bin/env python3
from threading import Event, Lock

import rospy
from apriltag_ros.msg import AprilTagDetectionArray
from std_msgs.msg import String

from bw_tags.srv import RequestTags, RequestTagsResponse
from bw_tools.typing.basic import get_param


class SimulatedTagRequestServer:
    def __init__(self) -> None:
        rospy.init_node("simulated_tag_request_server")

        self.timeout = get_param("~timeout", 5.0)

        self.simulated_tag_request_pub = rospy.Publisher("simulated_tag/request", String, queue_size=10)
        self.simulated_tag_response_sub = rospy.Subscriber(
            "simulated_tag/response", AprilTagDetectionArray, self.simulated_tag_request_callback
        )
        self.tags_request_srv = rospy.Service("tags_request", RequestTags, self.request_tags_callback)
        self.simulated_response: AprilTagDetectionArray = AprilTagDetectionArray()
        self.response_lock = Lock()
        self.response_event = Event()

    def request_tags_callback(self, request: RequestTags) -> RequestTagsResponse:
        self.send_request()
        response = RequestTagsResponse()
        self.response_event.wait(self.timeout)
        if self.response_event.is_set():
            with self.response_lock:
                response.tags = self.simulated_response
                self.response_event.clear()
                return response
        else:
            rospy.logwarn("Simulated tag request timed out")
            return response

    def send_request(self) -> None:
        self.simulated_tag_request_pub.publish("simulated_tag_request")

    def simulated_tag_request_callback(self, msg: AprilTagDetectionArray) -> None:
        with self.response_lock:
            self.simulated_response = msg
            self.response_event.set()

    def run(self) -> None:
        rospy.spin()


if __name__ == "__main__":
    node = SimulatedTagRequestServer()
    node.run()
