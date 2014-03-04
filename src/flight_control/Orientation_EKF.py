import serial
import sys
from math import atan2
import numpy as np
from EKF import Orientation_EKF
from time import sleep
import matplotlib.pyplot as plt

# Tunable
Q = np.mat ( ([5.0, 0, 0],
              [0, 100.0, 0],
              [0, 0, .01]) )
R = np.mat ( ([1000.0, 0],
              [0, 1000.0]) )
#Q = np.mat ( ([1.0, 0, 0],
              #[0, 100.0, 0],
              #[0, 0, .01]) )
#R = np.mat ( ([100.0, 0],
              #[0, 1000.0]) )

# Static 
H = np.mat ( ([1, 0, 0],
              [0, 1, 0]) )
B = np.zeros ((3,2))

pitch_filt = Orientation_EKF (B, Q, H, R, .01)
roll_filt = Orientation_EKF (B, Q, H, R, .01)

u = np.mat ( ([0],
              [0]) )

class vector:
    x = 0
    y = 0
    z = 0

accel_x = []
accel_y = []
accel_z = []
gyro_x = []
gyro_y = []
gyro_z = []

ser = serial.Serial ("/dev/ttymxc3", 115200)
print ser.name
print

pitch_record = []
roll_record = []

pitch_record_raw = []
roll_record_raw = []

accel = vector ()
gyro = vector ()

while True:
    try:
        # Getting Message
        msg = None
        msg = ser.readline ()
        msg = msg.split (":")

        accel_msg = msg[0].split (",")
        accel.x = float (accel_msg[0])
        accel.y = float (accel_msg[1])
        accel.z = float (accel_msg[2])

        gyro_msg = msg[1].split (",")
        gyro.x = float (gyro_msg[0])
        gyro.y = float (gyro_msg[1])
        gyro.z = float (gyro_msg[2])

        # Storing Raw Data
        accel_x.append (accel.x)
        accel_y.append (accel.y)
        accel_z.append (accel.z)

        gyro_x.append (gyro.x)
        gyro_y.append (gyro.y)
        gyro_z.append (gyro.z)

        acc_pitch = atan2 (accel.x, -accel.z)
        acc_roll = -atan2 (accel.y, -accel.z)
        
        pitch_record_raw.append (acc_pitch * (180/3.1415))
        roll_record_raw.append (acc_roll * (180/3.1415))

        # Start filtering here
        Z_pitch = np.mat ( ([acc_pitch],
                            [gyro.x]) )

        Z_roll = np.mat ( ([acc_roll],
                            [gyro.y]) )

        pitch = float (pitch_filt.compute (Z_pitch, u)[0][0])
        roll = float (roll_filt.compute (Z_roll, u)[0][0])

        # Storing filtered data
        pitch_record.append (pitch)
        roll_record.append (roll)
        
        sleep (.01)

    except KeyboardInterrupt:
        ser.close ()
        plt.plot (pitch_record, label="pitch")
        plt.plot (roll_record, label="roll")
        #plt.plot (pitch_record_raw, label="pitch_raw")
        #plt.plot (roll_record_raw, label="roll_raw")

        """
        plt.plot (accel_x, label="accel_x")
        plt.plot (accel_y, label="accel_y")
        plt.plot (accel_z, label="accel_z")
        plt.plot (gyro_x, label="gyro_x")
        plt.plot (gyro_y, label="gyro_y")
        plt.plot (gyro_z, label="gyro_z")
        """

        plt.legend()
        plt.show ()
