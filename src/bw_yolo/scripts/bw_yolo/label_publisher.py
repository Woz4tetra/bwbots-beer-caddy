#!/usr/bin/env python3
import rospy

from bw_interfaces.msg import Labels
from bw_tools.typing.basic import get_param


class LabelPublisher:
    def __init__(self) -> None:
        rospy.init_node("label_publisher")
        self.class_names_path = get_param("~class_names_path", "")
        self.publish_rate = get_param("~publish_rate", 5.0)
        self.class_names = self.read_class_names(self.class_names_path)
        self.label_msg = Labels()
        self.label_msg.labels = [str(x) for x in self.class_names]
        self.label_pub = rospy.Publisher("labels", Labels, queue_size=10)

    def read_class_names(self, path: str):
        if path and type(path) == str:
            rospy.loginfo(f"Loading class names from {path}")
            with open(path) as file:
                return file.read().splitlines()
        else:
            rospy.logwarn("No class names path defined. Not loading class names")
            return []

    def run(self):
        rate = rospy.Rate(self.publish_rate)
        while not rospy.is_shutdown():
            rate.sleep()
            self.label_pub.publish(self.label_msg)


def main():
    node = LabelPublisher()
    node.run()


if __name__ == "__main__":
    main()
