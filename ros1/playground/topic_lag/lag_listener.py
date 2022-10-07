#!/usr/bin/python3
import rospy

from zed_interfaces.msg import ObjectsStamped


class LagListener:
    def __init__(self):
        self.node_name = "lag_listener"
        rospy.init_node(
            self.node_name
            # disable_signals=True
            # log_level=rospy.DEBUG
        )

        self.sub = rospy.Subscriber("/zed/obj_det/yolo_objects", ObjectsStamped, self.callback, queue_size=1)

        rospy.loginfo("%s init complete" % self.node_name)

    def run(self):
        rospy.spin()

    def callback(self, msg):
        rospy.loginfo("delay: %0.4f" % (rospy.Time.now() - msg.header.stamp).to_sec())
    

if __name__ == "__main__":
    node = LagListener()
    try:
        node.run()

    except rospy.ROSInterruptException:
        pass

    finally:
        rospy.loginfo("Exiting %s node" % node.node_name)
