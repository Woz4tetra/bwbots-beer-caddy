#!/usr/bin/env python3
import rospy

from bw_yolo.srv import RequestFrame


def main():
    rospy.init_node("request_frame")
    detection_request = rospy.ServiceProxy("/bw/detection_request", RequestFrame)
    detections = detection_request()
    print(detections)


if __name__ == "__main__":
    main()
