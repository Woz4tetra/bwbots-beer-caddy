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

        self.midi_path = rospy.get_param("~midi_path", "simple.mid")
        self.volume = rospy.get_param("~volume", 30)
        self.midi_sequencer = MidiSequencer(self.midi_path, self.volume, True)
        self.gen = SequenceGenerator()

        rospy.loginfo("%s init complete" % self.node_name)

    def run(self):
        time.sleep(1.0)

        try:
            rospy.loginfo("Generating sequence")
            length = self.midi_sequencer.generate(self.gen)
            rospy.loginfo(f"Publishing sequence with {length} notes. Length is {len(self.gen.msg.sequence)}")
            self.sequence_pub.publish(self.gen.msg)
            rospy.loginfo("Starting sequence")
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
