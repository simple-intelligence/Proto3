import serial
import sys
from time import sleep
import matplotlib.pyplot as plt

"""
Accel X and Y data will be positive if the force is in the direction of the arrows on the diagram
Accel Z data will be negative when unit is held face up

"""

ser = serial.Serial ("/dev/ttymxc3", 115200)
print ser.name
print

class data:
    x = 0
    y = 0
    z = 0

accel_x = []
accel_y = []
accel_z = []
gyro_x = []
gyro_y = []
gyro_z = []

accel = data ()
gyro = data ()


while True:
    try:
        msg = None
        msg = ser.readline ()
        msg = msg.split (":")

        accel_msg = msg[0].split (",")
        accel.x = -float (accel_msg[0]) / (512 / 16)
        accel.y = float (accel_msg[1]) / (512 / 16)
        accel.z = -float (accel_msg[2]) / (512 / 16)

        gyro_msg = msg[2].split (",")
        gyro.x = float (gyro_msg[0]) / (14.375)
        gyro.y = float (gyro_msg[1]) / (14.375)
        gyro.z = float (gyro_msg[2]) / (14.375)

        accel_x.append (accel.x)
        accel_y.append (accel.y)
        accel_z.append (accel.z)

        # Check if x and z are switched
        gyro_x.append (gyro.x)
        gyro_y.append (gyro.y)
        gyro_z.append (gyro.z)

        sleep (.01)

    except ValueError or KeyError:
        pass

    except KeyboardInterrupt:
        ser.close ()
        #plt.plot (accel_x, label="accel_x")
        #plt.plot (accel_y, label="accel_y")
        #plt.plot (accel_z, label="accel_z")
        plt.plot (gyro_x, label="gyro_x")
        plt.plot (gyro_y, label="gyro_y")
        plt.plot (gyro_z, label="gyro_z")
        plt.legend ()
        plt.show ()
