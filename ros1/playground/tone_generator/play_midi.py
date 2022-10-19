#!/usr/bin/python3
import time
import rospy

import mido
from mido.midifiles.meta import MetaMessage

from bw_interfaces.msg import BwSequence
from bw_interfaces.srv import PlaySequence
from bw_interfaces.srv import StopSequence

from sequence_generator import SequenceGenerator
from midi_sequencer import MidiSequencer


def note_to_freq(note):
    a = 440 # frequency of A (common value is 440Hz)
    return (a / 32) * (2 ** ((note - 9) / 12))


class MidiPlayer:
    def __init__(self):
        self.node_name = "midi_player"
        rospy.init_node(
            self.node_name,
            disable_signals=True
            # log_level=rospy.DEBUG
        )

        self.sequence_pub = rospy.Publisher("/bw/sequence", BwSequence, queue_size=10)
        self.start_seq_srv = rospy.ServiceProxy("/bw/play_sequence", PlaySequence)
        self.stop_seq_srv = rospy.ServiceProxy("/bw/stop_sequence", StopSequence)

        self.midi_sequencer = MidiSequencer("megalovania.mid", 30, True)
        self.gen = SequenceGenerator()

        rospy.loginfo("%s init complete" % self.node_name)

    def run(self):
        time.sleep(1.0)

        try:
            self.midi_sequencer.generate(self.gen)
            self.sequence_pub.publish(self.gen.msg)
            print(self.start_seq_srv(self.gen.serial, False))
            rospy.spin()
        finally:
           self.stop_seq_srv() 

if __name__ == "__main__":
    node = MidiPlayer()
    try:
        node.run()

    except rospy.ROSInterruptException:
        pass

    finally:
        rospy.loginfo("Exiting %s node" % node.node_name)
