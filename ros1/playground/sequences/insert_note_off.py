from mido import Message, MidiFile, MidiTrack
from mido.midifiles.meta import MetaMessage

import sys
import time
from mido import MidiFile

filename = sys.argv[1]
if len(sys.argv) == 3:
    portname = sys.argv[2]
else:
    portname = None

try:
    midifile = MidiFile(filename)
    new_midi = MidiFile()
    t0 = time.time()
    for track in midifile.tracks:
        print(len(track))
    track = MidiTrack()
    new_midi.tracks.append(track)
    
    for count, msg in enumerate(midifile):
        if isinstance(msg, MetaMessage):
            continue
        if "note" not in msg.type:
            continue
        print(msg)
        if msg.type == "note_on":
            track.append(Message('note_on', note=msg.note, velocity=msg.velocity, time=0))
            track.append(Message('note_off', note=msg.note, velocity=msg.velocity, time=int(msg.time * 1000)))
    print('play time: {:.2f} s (expected {:.2f})'.format(
            time.time() - t0, midifile.length))
    new_midi.save("new_" + filename)

except KeyboardInterrupt:
    print()


