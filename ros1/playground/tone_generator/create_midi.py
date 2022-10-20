from mido import Message, MidiFile, MidiTrack

mid = MidiFile()
track = MidiTrack()
mid.tracks.append(track)

for _ in range(10):
    track.append(Message('note_on', note=55, velocity=64, time=0))
    track.append(Message('note_on', note=59, velocity=64, time=40))
    track.append(Message('note_on', note=62, velocity=64, time=80))
    track.append(Message('note_off', note=55, velocity=127, time=100))
    track.append(Message('note_off', note=59, velocity=127, time=0))
    track.append(Message('note_off', note=62, velocity=127, time=0))

mid.save('simple.mid')
