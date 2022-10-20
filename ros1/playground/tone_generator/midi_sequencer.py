import mido
from mido.midifiles.meta import MetaMessage
from sequence_generator import SequenceGenerator


def note_to_freq(note):
    a = 440 # frequency of A (common value is 440Hz)
    return (a / 32) * (2 ** ((note - 9) / 12))


class MidiSequencer:
    def __init__(self, file_path, max_volume, kick_old_tones=False, max_length=None) -> None:
        self.kick_old_tones = kick_old_tones
        self.max_volume = max_volume
        self.midi = mido.MidiFile(file_path, clip=True)
        self.max_length = max_length
        self.active_notes = {
            0: None,
            1: None,
            2: None,
            # 3: None,
        }

    def is_channel_playable(self, channel):
        return channel <= 8

    def generate(self, generator: SequenceGenerator):
        channel_queue = []
        count = 0
        initial_tempo = None
        tempo_multiplier = 1.0
        for count, msg in enumerate(self.midi):
            if self.max_length is not None and count >= self.max_length:
                break
            if isinstance(msg, MetaMessage):
                if msg.type == "set_tempo":
                    if initial_tempo is None:
                        initial_tempo = msg.tempo
                    tempo_multiplier = msg.tempo / initial_tempo
                    print(f"tempo: {msg} multiplier: {tempo_multiplier}")
                else:
                    print("ignoring:", msg)
                    continue
            
            # if "note" not in msg.type:
            #     print("ignoring:", msg)
            #     continue
            if "time" in msg.dict():
                print(msg)
                delay_ms = int(msg.time * 1000 * tempo_multiplier)
                if msg.time > 0.0 and delay_ms == 0:
                    delay_ms = 1
                
                if delay_ms > 0:
                    print(f"{len(generator)}: Delay {delay_ms} ms. {msg}")
                    generator.add(SequenceGenerator.make_delay(delay_ms))

            if "note" in msg.type:
                frequency = note_to_freq(msg.note)
                while frequency < 20:
                    frequency *= 2

                if not self.is_channel_playable(msg.channel):
                    continue
            else:
                frequency = 0
            

            if msg.type == "note_on":
                note_added = False
                for channel, note in self.active_notes.items():
                    if note is None:
                        self.active_notes[channel] = msg
                        print(f"{len(generator)}: On {frequency} Hz on channel {channel}. {msg}")
                        generator.add(
                            SequenceGenerator.make_start_tone(channel, frequency, self.max_volume)
                        )
                        note_added = True
                        channel_queue.append(channel)
                        if len(channel_queue) >= len(self.active_notes):
                            channel_queue.pop(0)
                        break
                if not note_added and self.kick_old_tones:
                    kick_channel = channel_queue.pop(0)
                    channel_queue.append(kick_channel)
                    print(f"{len(generator)}: On {frequency} Hz on channel {channel}. {msg}")
                    SequenceGenerator.make_start_tone(kick_channel, frequency, self.max_volume)
                    self.active_notes[kick_channel] = msg
                
            elif msg.type == "note_off":
                for channel, note in self.active_notes.items():
                    if note is None:
                        continue
                    if msg.note == note.note:
                        self.active_notes[channel] = None
                        print(f"{len(generator)}: Off {frequency} Hz on channel {channel}. {msg}")
                        generator.add(SequenceGenerator.make_stop_tone(channel))
                        break
        return count
