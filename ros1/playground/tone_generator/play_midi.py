#!/usr/bin/python3
import time
import rospy

import mido

from bw_interfaces.msg import BwDriveTone


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

        self.tone_pub = rospy.Publisher("/bw/module_tone", BwDriveTone, queue_size=10)

        self.midi = mido.MidiFile('megalovania.mid', clip=True)

        self.channel_map = {
            0: 0,
            1: 1,
            # 9: 3
        }

        rospy.loginfo("%s init complete" % self.node_name)

    def publish_tone(self, channel, frequency, volume):
        msg = BwDriveTone()
        msg.module_index = str(int(channel))
        msg.frequency = int(frequency)
        msg.speed = int(volume)
        self.tone_pub.publish(msg)
    
    def stop_tone(self):
        self.publish_tone(0, 0, 0)
    
    def run(self):
        time.sleep(2.0)

        try:
            for msg in self.midi.play():
                print(msg)
                time.sleep(msg.time)
                if msg.type != "note_on":
                    continue
                if msg.channel not in self.channel_map:
                    continue
                channel = self.channel_map[msg.channel]
                frequency = note_to_freq(msg.note)
                while frequency < 100:
                    frequency *= 2

                self.publish_tone(channel, frequency, 30)
        finally:
            self.stop_tone()

if __name__ == "__main__":
    node = MidiPlayer()
    try:
        node.run()

    except rospy.ROSInterruptException:
        pass

    finally:
        rospy.loginfo("Exiting %s node" % node.node_name)
