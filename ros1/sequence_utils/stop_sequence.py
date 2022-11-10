#!/usr/bin/python3
import rospy

from bw_interfaces.srv import StopSequence


if __name__ == "__main__":
    rospy.init_node(
        "stop_sequence_script",
        disable_signals=True
        # log_level=rospy.DEBUG
    )

    stop_seq_srv = rospy.ServiceProxy("/bw/stop_sequence", StopSequence)
    stop_seq_srv()
