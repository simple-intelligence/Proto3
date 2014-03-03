import numpy as np
from EKF import Orientation_EKF
from math import atan2

B = np.zeros ((3,2))
Q = np.mat ( ([.1, 0, 0],
              [0, .1, 0],
              [0, 0, .1]) )
H = np.mat ( ([1, 0, 0],
              [0, 1, 0]) )
R = np.mat ( ([.1, 0],
              [0, .1]) )

pitch_filt = Orientation_EKF (B, Q, H, R)
roll_filt = Orientation_EKF (B, Q, H, R)

acc_pitch = atan2 (0, -9.8)
acc_roll = -atan2 (0, -9.8)

Z_pitch = np.mat ( ([acc_pitch],
                    [    10   ]) )
Z_roll = np.mat ( ([acc_roll],
                    [    10   ]) )

u = np.mat ( ([0],
              [0]) )

print pitch_filt.compute (Z_pitch, u)
print roll_filt.compute (Z_roll, u)
