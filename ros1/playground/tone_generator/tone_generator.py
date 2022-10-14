#!/usr/bin/python3
import time
import rospy

from bw_interfaces.msg import BwDriveTone

class ToneGenerator:
    def __init__(self):
        self.node_name = "tone_generator"
        rospy.init_node(
            self.node_name,
            disable_signals=True
            # log_level=rospy.DEBUG
        )

        self.tone_pub = rospy.Publisher("/bw/module_tone", BwDriveTone, queue_size=10)

        self.semitone_table = {
            "C3": 131,
            "C#3": 139,
            "Db3": 139,
            "D3": 147,
            "D#3": 156,
            "Eb3": 156,
            "E3": 164,
            "F3": 175,
            "F#3": 185,
            "Gb3": 185,
            "G3": 196,
            "Ab3": 208,
            "A3": 220,
            "A#3": 233,
            "Bb3": 233,
            "B3": 247,
            "C4": 262,
            "C#4": 277,
            "Db4": 277,
            "D4": 294,
            "D#4": 311,
            "Eb4": 311,
            "E4": 330,
            "F4": 349,
            "F#4": 370,
            "Gb4": 370,
            "G4": 392,
            "G#4": 415,
            "Ab4": 415,
            "A4": 440,
            "B4": 466,
            "C5": 493,
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
    
    def play_song(self, channel, volume, song: str):
        song = song.strip()
        try:
            for note in song.split(" "):
                name, duration = note.split(",")
                duration = int(duration) / 1000.0
                frequency = self.semitone_table[name]

                self.publish_tone(channel, frequency, volume)
                time.sleep(duration)
        finally:
            self.stop_tone()

    def run(self):
        time.sleep(2.0)
        # self.play_song(0, 30, "G3,250 A3,250 B3,250 C4,250 D4,250 E4,250 F4,250 G4,500 F4,250 E4,250 D4,250 C4,250 B3,250 A3,250 G3,500")
        self.play_song(0, 30, "Eb3,250 G3,250 F3,250 Eb3,250 F3,250 G3,250 Ab3,250 Bb3,250 C4,250 D4,250 Eb4,250 D4,250 C4,250 Bb3,250 Ab3,250 G3,250 F3,250 Eb3,250 G3,250 F3,250 Eb3,500")


if __name__ == "__main__":
    node = ToneGenerator()
    try:
        node.run()

    except rospy.ROSInterruptException:
        pass

    finally:
        rospy.loginfo("Exiting %s node" % node.node_name)
