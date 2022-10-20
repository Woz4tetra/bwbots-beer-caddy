#!/usr/bin/python3
import time
import rospy

from bw_interfaces.srv import PlaySequence
from bw_interfaces.srv import StopSequence


class LastSequencePlayer:
    def __init__(self):
        self.node_name = "midi_player"
        rospy.init_node(
            self.node_name,
            disable_signals=True
            # log_level=rospy.DEBUG
        )

        self.start_seq_srv = rospy.ServiceProxy("/bw/play_sequence", PlaySequence)
        self.stop_seq_srv = rospy.ServiceProxy("/bw/stop_sequence", StopSequence)

        self.serial = int(rospy.get_param("~serial", 0))

        rospy.loginfo("%s init complete" % self.node_name)

    def run(self):
        time.sleep(1.0)

        try:
            rospy.loginfo("Starting sequence")
            print(self.start_seq_srv(self.serial, False))
            rospy.spin()
        finally:
           self.stop_seq_srv() 

if __name__ == "__main__":
    node = LastSequencePlayer()
    try:
        node.run()

    except rospy.ROSInterruptException:
        pass

    finally:
        rospy.loginfo("Exiting %s node" % node.node_name)
