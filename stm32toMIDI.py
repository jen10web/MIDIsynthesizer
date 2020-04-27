import serial
import time
import rtmidi2

notes = [40,55,71]

midi_out = rtmidi2.MidiOut()
midi_out.open_port("loopMIDI Port 3")

input("press enter to start")

ser = serial.Serial(
    port='COM7',\
    baudrate=115200,\
    parity=serial.PARITY_NONE,\
    stopbits=serial.STOPBITS_ONE,\
    bytesize=serial.EIGHTBITS,\
        timeout=0)

print("connected to: " + ser.portstr)

playing = 0
last = None

while True:
    for c in ser.read():
        if last != c:
            last = c
            continue
        if c == 49:
            if playing != 49:
                midi_out.send_noteon(0, notes[0], 100)
                print("Recieved keypad position 1 - sending MIDI note")
                playing = 49
        elif c == 50:
            if playing != 50:   
                midi_out.send_noteon(0, notes[1], 100)
                print("Recieved keypad position 2 - sending MIDI note")
                playing = 50
        elif c == 51:
            if playing != 51: 
                midi_out.send_noteon(0, notes[2], 100)
                print("Recieved keypad position 3 - sending MIDI note")
                playing = 51
        elif playing != 0:
            midi_out.send_noteoff(0, notes[0])
            midi_out.send_noteoff(0, notes[1])
            midi_out.send_noteoff(0, notes[2])
            playing = 0
        last = c

ser.close()
