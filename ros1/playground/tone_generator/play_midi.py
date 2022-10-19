#!/usr/bin/python3
import time
import rospy

import mido
from mido.midifiles.meta import MetaMessage

from bw_interfaces.msg import BwSequence
from bw_interfaces.srv import PlaySequence
from bw_interfaces.srv import StopSequence

from sequence_generator import SequenceGenerator


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

        self.gen = SequenceGenerator()

        self.midi = mido.MidiFile('megalovania.mid', clip=True)

        self.channel_map = {
            0: 0,
            1: 1,
            # 9: 3
        }

        rospy.loginfo("%s init complete" % self.node_name)

    def run(self):
        time.sleep(2.0)

        try:
            for msg in self.midi:
                print(msg)
                if isinstance(msg, MetaMessage):
                    continue
                if msg.type != "note_off":
                    continue
                if msg.channel not in self.channel_map:
                    continue
                channel = self.channel_map[msg.channel]
                if channel != 0:
                    continue
                frequency = note_to_freq(msg.note)
                while frequency < 100:
                    frequency *= 2

                self.gen.add(*self.gen.make_tone(frequency, 30, msg.time * 1000))
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
