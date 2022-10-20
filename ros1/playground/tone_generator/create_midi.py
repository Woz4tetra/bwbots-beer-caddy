from mido import Message, MidiFile, MidiTrack

mid = MidiFile()
track = MidiTrack()
mid.tracks.append(track)

track.append(Message('note_on', note=55, velocity=64, time=0))
track.append(Message('note_on', note=59, velocity=64, time=0))
track.append(Message('note_on', note=62, velocity=64, time=0))
track.append(Message('note_off', note=55, velocity=127, time=500))
track.append(Message('note_off', note=59, velocity=127, time=0))
track.append(Message('note_off', note=62, velocity=127, time=0))

mid.save('simple.mid')
