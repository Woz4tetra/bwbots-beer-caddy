import mido
import colorsys
from mido.midifiles.meta import MetaMessage
from .sequence_generator import SequenceGenerator


def note_to_freq(note):
    a = 440 # frequency of A (common value is 440Hz)
    frequency = (a / 32) * (2 ** ((note - 9) / 12))
    while frequency < 20:
        frequency *= 2
    return frequency


class MidiSequencer:
    def __init__(
            self,
            file_path,
            max_volume,
            channel_dithering=0,
            dither_delay=0.1,
            tempo_multiplier=1.0,
            allowed_tracks=None,
            num_leds=None,
            num_channels=4) -> None:
        # channel_dithering == 0 means kick off old tones if one is queued
        # channel_dithering == 1 means old tones having priority over new tones
        # channel_dithering >= 2 means channels will flutter between N queued tones
        self.channel_dithering = channel_dithering
        self.dither_delay = dither_delay
        self.max_volume = max_volume
        self.tempo_multiplier = 1.0 / tempo_multiplier
        self.midi = mido.MidiFile(file_path, clip=True)
        self.max_midi_volume = 127
        self.allowed_tracks = allowed_tracks
        self.num_leds = num_leds
        self.num_channels = num_channels
        self.show_delay = 6  # millisecond delay introduced by show command

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

    def get_volume(self, velocity):
        return int(velocity / self.max_midi_volume * self.max_volume)

    def generate(self, generator: SequenceGenerator):
        channel_queue = []
        count = 0
        initial_tempo = None
        relative_tempo = 1.0
        active_notes = {channel: [] for channel in range(self.num_channels)}
        dither_delay_ms = max(1, int(1000.0 * self.dither_delay))
    
        # set all LEDs to their "note off" state
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
                    # the only relevant meta data is set_tempo
                    # if tempo is only set once in the song, it's ignored.
                    # if tempo is changed in the song, delays are multiplied
                    # by new tempo divided by initial tempo
                    if initial_tempo is None:
                        initial_tempo = msg.tempo
                    relative_tempo = msg.tempo / initial_tempo
                    print(f"relative tempo: {msg}: {relative_tempo}")
                else:
                    print("ignoring:", msg)
                    continue

            if "time" in msg.dict():
                print(msg)
                # get delay associated with note or other message type.
                # multiply time by relative tempo change. Convert to milliseconds.
                delay_ms = int(msg.time * 1000 * relative_tempo * self.tempo_multiplier)
                if msg.time > 0.0 and delay_ms == 0:
                    # if time delay is not zero and less than 1 ms, set to 1 ms
                    delay_ms = 1
                
                if delay_ms > 0:
                    can_add_show = self.num_leds is not None and delay_ms > self.show_delay + 1
                    if can_add_show:
                        # there is delay associated with drawing the LED state.
                        # if LEDs are enabled for this song and if the delay is longer than
                        # the time it takes to update the LED, offset the requested delay by
                        # the LED draw delay
                        delay_ms -= self.show_delay
                    
                    # if more than one note is queued on a channel, we need to dither between
                    # the two notes.
                    no_dithering = all([len(notes) <= 1 for notes in active_notes.values()])
                    if no_dithering or delay_ms <= dither_delay_ms:
                        # if dithering is not required, queue a delay like normal
                        generator.add(SequenceGenerator.make_delay(delay_ms))
                        print(f"{len(generator)}: Delay {delay_ms} ms. {msg}")
                    else:
                        # track the last note that was played
                        dither_tracker = {channel: 0 for channel in range(len(active_notes))}

                        # subtract approximate time it takes to switch tones
                        for _ in range(int(delay_ms / dither_delay_ms)):
                            # use dither frequency as delay
                            generator.add(SequenceGenerator.make_delay(dither_delay_ms))

                            for channel, notes in active_notes.items():
                                if len(notes) == 0:
                                    continue

                                # select a note
                                selected_note = dither_tracker[channel]
                                
                                # move to the next note. Loop back if the end is reached
                                dither_tracker[channel] = (selected_note + 1) % len(notes)
                                
                                # get note data
                                note_msg = notes[selected_note]
                                
                                # add note (firmware overwrites previous note automatically)
                                frequency = note_to_freq(note_msg.note)
                                volume = self.get_volume(note_msg.velocity)
                                generator.add(SequenceGenerator.make_start_tone(channel, frequency, volume))
                        remainder = int(delay_ms % dither_delay_ms)
                        if remainder > 0:
                            generator.add(SequenceGenerator.make_delay(remainder))

                    # add show command if required
                    if can_add_show:
                        generator.add(SequenceGenerator.make_show())

            if "note" in msg.type:
                # extract note meta data
                frequency = note_to_freq(msg.note)
                if not self.is_channel_playable(msg.channel):
                    continue
                
                volume = self.get_volume(msg.velocity)
                if volume <= 0:
                    note_type = "note_off"
                else:
                    note_type = msg.type

            else:
                # this message is not a note, set null values
                frequency = 0
                volume = 0
                note_type = ""
            

            if note_type == "note_on":
                channel_of_added_note = -1  # -1 == no note added

                for channel, notes in active_notes.items():
                    if len(notes) > 0:  # a note is already on this channel, handle this below
                        continue
                    
                    # track note as active on this channel
                    active_notes[channel].append(msg)
                    print(f"{len(generator)}: On {frequency} Hz on channel {channel}. {msg}")
                    generator.add(
                        SequenceGenerator.make_start_tone(channel, frequency, volume),
                        self.led_from_note(msg.note, True)
                    )
                    
                    # track what channel this note is played on
                    channel_of_added_note = channel
                    break
                if channel_of_added_note == -1:
                    if self.channel_dithering == 0:
                        # channel_dithering == 0 means kick off old tones if one is queued
                        # rotate over kicked channels to spread out which notes get interrupted
                        kick_channel = channel_queue[0]
                        if all([msg.note > active_msg.note for active_msg in active_notes[kick_channel]]):
                            # if the new note's frequency is higher than all the active notes,
                            # kick all active notes in favor of the new note
                            kick_channel = channel_queue.pop(0)
                            channel_queue.append(kick_channel)
                            print(f"{len(generator)}: On {frequency} Hz overwrote old on channel {channel}. {msg}")
                            generator.add(
                                SequenceGenerator.make_start_tone(kick_channel, frequency, volume),
                                self.led_from_note(msg.note, True)
                            )
                            channel_of_added_note = kick_channel
                            
                            # remove all active notes
                            while len(active_notes[kick_channel]) > 0:
                                active_notes[kick_channel].pop(0)
                            
                            # add the new note
                            active_notes[kick_channel].append(msg)
                    # channel_dithering == 1 means old tones having priority over new tones. do nothing
                    elif self.channel_dithering > 1:
                        # channel_dithering >= 2 means channels will flutter between N queued tones
                        
                        # select the channel with the least number of active notes
                        dither_channel = -1
                        min_active = None
                        for channel, notes in active_notes.items():
                            num_active = len(notes)
                            if min_active is None or num_active < min_active:
                                min_active = num_active
                                dither_channel = channel

                        # if we've maxed out the number of allowable dithering notes, skip this note
                        if dither_channel != -1 and len(active_notes[dither_channel]) < self.channel_dithering:
                            # otherwise, add to the map of active_notes. Dithering will start on the next delay
                            active_notes[dither_channel].append(msg)
                            print(f"{len(generator)}: On {frequency} Hz dither on channel {dither_channel}. {msg}")
                            channel_of_added_note = dither_channel

                if channel_of_added_note != -1:
                    # if a note was added, rotate the kicking/dithering queue
                    channel_queue.append(channel_of_added_note)
                    if len(channel_queue) >= len(active_notes):
                        channel_queue.pop(0)
                
            elif note_type == "note_off":
                notes_to_remove = []  # track which notes were removed. (channel, note_msg)
                for channel, note_msgs in active_notes.items():
                    # if there are no active notes on this channel, there's nothing to remove/stop
                    if len(note_msgs) == 0:
                        continue
                    note_found = False
                    for active_msg in note_msgs:
                        # if the requested note to stop, is one of the active notes,
                        # remove it and queue a stop tone message
                        if msg.note == active_msg.note:
                            note_found = True
                            notes_to_remove.append((channel, active_msg))
                            print(f"{len(generator)}: Off {frequency} Hz on channel {channel}. {msg}")
                            generator.add(
                                SequenceGenerator.make_stop_tone(channel),
                                self.led_from_note(msg.note, False)
                            )
                            break
                    if note_found:
                        break
                # delete notes that were stopped
                for channel, note_msg in notes_to_remove:
                    active_notes[channel].remove(note_msg)
                del notes_to_remove

        if self.num_leds is not None:
            for index in range(self.num_leds):
                generator.add(
                    SequenceGenerator.make_led(index, 0, 0, 0, 0),
                    SequenceGenerator.make_show()
                )
        return count
