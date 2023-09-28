#!/usr/bin/env python3
import rospy

from bw_tags.srv import RequestTags


def main():
    rospy.init_node("request_frame")
    tags_request = rospy.ServiceProxy("/bw/tags_request", RequestTags)
    tags = tags_request()
    print(tags)


if __name__ == "__main__":
    main()
