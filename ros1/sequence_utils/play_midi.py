#!/usr/bin/python3
import time
import rospy

from bw_interfaces.msg import BwSequence
from bw_interfaces.srv import PlaySequence

from bw_tools.sequencers import MidiSequencer
from bw_tools.sequencers import SequenceGenerator


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

        self.midi_path = rospy.get_param("~midi_path", "midi/simple.mid")
        self.volume = rospy.get_param("~volume", 30)
        self.tempo_multiplier = rospy.get_param("~tempo_multiplier", 1.0)
        self.loop = rospy.get_param("~loop", False)
        self.allowed_tracks = rospy.get_param("~allowed_tracks", [0, 1, 2, 3, 4, 5, 6, 7])
        self.num_leds = rospy.get_param("~num_leds", 24)
        self.channel_dithering = rospy.get_param("~channel_dithering", 0)
        self.dither_delay = rospy.get_param("~dither_delay", 0.1)
        self.num_channels = rospy.get_param("~num_channels", 4)
        self.midi_sequencer = MidiSequencer(
            self.midi_path,
            self.volume,
            self.channel_dithering,
            self.dither_delay,
            self.tempo_multiplier,
            self.allowed_tracks,
            self.num_leds,
            self.num_channels
        )
        self.gen = SequenceGenerator()

        rospy.loginfo("%s init complete" % self.node_name)

    def run(self):
        time.sleep(0.25)

        rospy.loginfo("Generating sequence")
        length = self.midi_sequencer.generate(self.gen)
        rospy.loginfo(f"Publishing sequence with {length} notes. Length is {len(self.gen.msg.sequence)}")
        self.sequence_pub.publish(self.gen.msg)
        rospy.loginfo("Starting sequence")
        print(self.start_seq_srv(self.gen.serial, self.loop, False))

if __name__ == "__main__":
    node = MidiPlayer()
    try:
        node.run()

    except rospy.ROSInterruptException:
        pass

    finally:
        rospy.loginfo("Exiting %s node" % node.node_name)
