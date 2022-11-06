import mido
import colorsys
from mido.midifiles.meta import MetaMessage
from .sequence_generator import SequenceGenerator


def note_to_freq(note):
    a = 440 # frequency of A (common value is 440Hz)
    return (a / 32) * (2 ** ((note - 9) / 12))


class MidiSequencer:
    def __init__(self, file_path, max_volume, kick_old_tones=False, tempo_multiplier=1.0, allowed_tracks=None, num_leds=None) -> None:
        self.kick_old_tones = kick_old_tones
        self.max_volume = max_volume
        self.tempo_multiplier = 1.0 / tempo_multiplier
        self.midi = mido.MidiFile(file_path, clip=True)
        self.active_notes = {
            0: None,
            1: None,
            2: None,
            3: None,
        }
        self.max_midi_volume = 127
        self.allowed_tracks = allowed_tracks
        self.num_leds = num_leds
        self.show_delay = 10  # millisecond delay introduced by show command

    def is_channel_playable(self, channel):
        if self.allowed_tracks is None or len(self.allowed_tracks) == 0:
            return True
        else:
            return channel in self.allowed_tracks

    def make_led(self, index, state):
        color = colorsys.hsv_to_rgb(index / self.num_leds, 1.0, 1.0)
        if state:
            r = int(color[0] * 200)
            g = int(color[1] * 200)
            b = int(color[2] * 200)
            w = 100
        else:
            r = int(color[0] * 50)
            g = int(color[1] * 50)
            b = int(color[2] * 50)
            w = 0
        
        return SequenceGenerator.make_led(index, r, g, b, w)

    def led_from_note(self, midi_note, state):
        if self.num_leds is None:
            return None
        index = midi_note % self.num_leds
        return self.make_led(index, state)

    def generate(self, generator: SequenceGenerator):
        channel_queue = []
        count = 0
        initial_tempo = None
        relative_tempo = 1.0
        if self.num_leds is not None:
            for index in range(self.num_leds):
                generator.add(
                    self.make_led(index, False),
                    SequenceGenerator.make_show()
                )
            generator.add(
                SequenceGenerator.make_delay(self.show_delay * self.num_leds)
            )

        for count, msg in enumerate(self.midi):
            if isinstance(msg, MetaMessage):
                if msg.type == "set_tempo":
                    if initial_tempo is None:
                        initial_tempo = msg.tempo
                    relative_tempo = msg.tempo / initial_tempo
                    print(f"relative tempo: {msg}: {relative_tempo}")
                else:
                    print("ignoring:", msg)
                    continue

            if "time" in msg.dict():
                print(msg)
                delay_ms = int(msg.time * 1000 * relative_tempo * self.tempo_multiplier)
                if msg.time > 0.0 and delay_ms == 0:
                    delay_ms = 1
                
                if delay_ms > 0:
                    print(f"{len(generator)}: Delay {delay_ms} ms. {msg}")
                    if self.num_leds is not None and delay_ms > self.show_delay + 1:
                        generator.add(SequenceGenerator.make_delay(delay_ms - self.show_delay))
                        generator.add(SequenceGenerator.make_show())
                    else:
                        generator.add(SequenceGenerator.make_delay(delay_ms))

            if "note" in msg.type:
                frequency = note_to_freq(msg.note)
                while frequency < 20:
                    frequency *= 2

                if not self.is_channel_playable(msg.channel):
                    continue
                    
                volume = int(msg.velocity / self.max_midi_volume * self.max_volume)
                if volume <= 0:
                    note_type = "note_off"
                else:
                    note_type = msg.type

            else:
                frequency = 0
                volume = 0
                note_type = ""
            

            if note_type == "note_on":
                note_added = False
                for channel, note in self.active_notes.items():
                    if note is None:
                        self.active_notes[channel] = msg
                        print(f"{len(generator)}: On {frequency} Hz on channel {channel}. {msg}")
                        generator.add(
                            SequenceGenerator.make_start_tone(channel, frequency, volume),
                            self.led_from_note(msg.note, True)
                        )
                        note_added = True
                        channel_queue.append(channel)
                        if len(channel_queue) >= len(self.active_notes):
                            channel_queue.pop(0)
                        break
                if not note_added and self.kick_old_tones:
                    kick_channel = channel_queue[0]
                    if msg.note > self.active_notes[kick_channel].note:
                        kick_channel = channel_queue.pop(0)
                        channel_queue.append(kick_channel)
                        print(f"{len(generator)}: On {frequency} Hz on channel {channel}. {msg}")
                        generator.add(
                            SequenceGenerator.make_start_tone(kick_channel, frequency, volume),
                            self.led_from_note(msg.note, True)
                        )
                        self.active_notes[kick_channel] = msg
                
            elif note_type == "note_off":
                for channel, note in self.active_notes.items():
                    if note is None:
                        continue
                    if msg.note == note.note:
                        self.active_notes[channel] = None
                        print(f"{len(generator)}: Off {frequency} Hz on channel {channel}. {msg}")
                        generator.add(
                            SequenceGenerator.make_stop_tone(channel),
                            self.led_from_note(msg.note, False)
                        )
                        break
        if self.num_leds is not None:
            for index in range(self.num_leds):
                generator.add(
                    SequenceGenerator.make_led(index, 0, 0, 0, 0),
                    SequenceGenerator.make_show()
                )
        return count
