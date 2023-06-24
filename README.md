# 8080
Automatic Acoustic Drumset "8080" - Arduino (tested on Arduino Mega)

Uses MD_MIDIFILE library to read MIDI files from an SD card.
After processing the midi file, the system moves a number of drumsticks across drums to drum in different positions and drums (according to the midi file).
Each note in the midi file represents a different drum (only the NOTE_ON event is parsed).
The BPM of the system is controlled by a potentiometer and a pedal.
The pedal is used to "drum with your foot" - according to your rhythm a bpm is interperted and the system changes drumming speed accordingly.

8080 can be used as an automatic drumming accompaniment system.
