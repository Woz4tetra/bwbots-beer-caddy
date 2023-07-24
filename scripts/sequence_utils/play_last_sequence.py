#!/usr/bin/python3
import time
import rospy

from bw_interfaces.srv import PlaySequence


class LastSequencePlayer:
    def __init__(self):
        self.node_name = "midi_player"
        rospy.init_node(
            self.node_name,
            disable_signals=True
            # log_level=rospy.DEBUG
        )

        self.start_seq_srv = rospy.ServiceProxy("/bw/play_sequence", PlaySequence)

        self.serial = int(rospy.get_param("~serial", 0))
        self.from_flash = int(rospy.get_param("~from_flash", True))

        rospy.loginfo("%s init complete" % self.node_name)

    def run(self):
        time.sleep(1.0)

        rospy.loginfo("Starting sequence")
        print(self.start_seq_srv(self.serial, False, self.from_flash))

if __name__ == "__main__":
    node = LastSequencePlayer()
    node.run()
