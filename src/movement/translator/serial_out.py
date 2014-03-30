import os
import sys
from time import sleep

#import Adafruit_BBIO.UART as UART
import serial

sys.path.append (os.path.abspath("../../"))
from communication.zmq_communicator import communicator

#ser = serial.Serial (port = "/dev/ttyACM0", baudrate=9600, timeout=0)
ser = serial.Serial (port = "/dev/ttyACM0", baudrate=9600)

def setup_serial ():
    ser.close ()
    ser.open ()

def send_flight_controls (pitch, yaw, roll, z, arm, stabalize, calibrate):
    if ser.isOpen ():
        ser.write ("B{p},{y},{r},{z_pos},{a},{s},{c}\n".format (p=int (pitch), y=int (yaw), r=int (roll), z_pos=int (z), a=int (arm), s=int (stabalize), c=int (calibrate)))
        print "B{p},{y},{r},{z_pos},{a},{s},{c}\n".format (p=int (pitch), y=int (yaw), r=int (roll), z_pos=int (z), a=int (arm), s=int (stabalize), c=int (calibrate))

def main ():
    com = communicator ("Translator")
    last_timestamp = 0

    while True:
        msg = com.get_message ("Switcher")
        if msg and msg["time"] > last_timestamp:
            last_timestamp = msg["time"]
            msg = msg["message"]

            send_flight_controls (msg["Pitch"], msg["Yaw"], msg["Roll"], msg["Z"], msg["Arm"], msg["Stabalize"], msg["Calibrate"])

        sleep (.05)

if __name__=="__main__":
    setup_serial ()
    main ()
