#!/usr/bin/env python
"""
Open a MIDI file and print every message in every track.
Support for MIDI files is still experimental.
"""
import sys
from mido import MidiFile
from mido.midifiles.meta import MetaMessage

if __name__ == '__main__':
    filename = sys.argv[1]

    midi_file = MidiFile(filename)

    for i, track in enumerate(midi_file.tracks):
        print(f"=== Track {i}")
        info = {
            "num_notes": 0,
            "length": 0
        }
        for message in track:
            if type(message) == MetaMessage:
                print(f"    {message}")
            else:
                if "note" in message.type:
                    info["num_notes"] += 1
            info["length"] += 1
        print(f"    Number of notes: {info['num_notes']}")
        print(f"    Length: {info['length']}")
