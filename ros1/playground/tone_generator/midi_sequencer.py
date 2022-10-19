import mido
from mido.midifiles.meta import MetaMessage
from sequence_generator import SequenceGenerator


def note_to_freq(note):
    a = 440 # frequency of A (common value is 440Hz)
    return (a / 32) * (2 ** ((note - 9) / 12))


class MidiSequencer:
    def __init__(self, file_path, max_volume, kick_old_tones=False) -> None:
        self.kick_old_tones = kick_old_tones
        self.max_volume = max_volume
        self.midi = mido.MidiFile(file_path, clip=True)
        self.active_notes = {
            0: None,
            1: None,
            2: None,
        }

    def generate(self, generator: SequenceGenerator):
        channel_queue = []
        for msg in self.midi:
            if isinstance(msg, MetaMessage):
                continue
            
            if "note" in msg.type:
                frequency = note_to_freq(msg.note)
                while frequency < 20:
                    frequency *= 2

                if msg.type == "note_on":
                    note_added = False
                    for channel, note in self.active_notes.items():
                        if note is None:
                            self.active_notes[channel] = msg
                            generator.add(
                                SequenceGenerator.make_start_tone(channel, frequency, self.max_volume)
                            )
                            note_added = True
                            channel_queue.append(channel)
                            if len(channel_queue) >= len(self.active_notes):
                                channel_queue.pop(0)
                    if not note_added and self.kick_old_tones:
                        kick_channel = channel_queue.pop(0)
                        channel_queue.append(kick_channel)
                        SequenceGenerator.make_start_tone(kick_channel, frequency, self.max_volume)
                        self.active_notes[kick_channel] = msg
                    if msg.delay > 0:
                        generator.add(SequenceGenerator.make_delay(msg.delay * 1000))

                elif msg.type == "note_off":
                    if msg.delay > 0:
                        generator.add(SequenceGenerator.make_delay(msg.delay * 1000))
                    for channel, note in self.active_notes.items():
                        if msg.note == note.note:
                            self.active_notes[channel] = None
                            generator.add(SequenceGenerator.make_stop_tone(channel))

