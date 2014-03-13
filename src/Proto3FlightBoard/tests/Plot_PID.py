import serial
import sys
from time import sleep
import matplotlib.pyplot as plt

ser = serial.Serial ("/dev/ttyACM0", 9600)
print ser.name
print

raw_pitch_record = []
raw_roll_record = []

pitch_record = []
roll_record = []

msg = None

while ser.isOpen():
    try:
        msg = ser.readline ()
        msg = msg.split (":")
        
        raw_pitch_record.append (float (msg[0]))
        raw_roll_record.append (float (msg[1]))
        pitch_record.append (float (msg[2]))
        roll_record.append (float (msg[3]))
        
        sleep (.01)

    except ValueError or KeyError:
        pass

    except KeyboardInterrupt:
        ser.close ()
        #plt.plot (raw_pitch_record, label="raw_pitch")
        #plt.plot (raw_roll_record, label="raw_roll")
        plt.plot (pitch_record, label="pitch_pid")
        plt.plot (roll_record, label="roll_pid")
        plt.legend ()
        plt.show ()
