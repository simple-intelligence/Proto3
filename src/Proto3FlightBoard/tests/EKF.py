import numpy as np
from time import time

class Orientation_EKF:
    def __init__(self, B, Q, H, R, dt):
        """
        For a quadcopter only! Not generalizable.
        """

        # Contant matrices
        self.B = B
        self.Q = Q
        self.H = H
        self.R = R
        
        self.init_x ()
        self.init_y ()
        self.init_P ()
        self.init_w ()
        self.init_v ()

        self.dt = dt

        self.update_F ()

        self.previous_time = 0
        self.time = 0

    def init_x (self):
        self.x =  np.mat ( ([0],
                            [0],
                            [0]) )
        self.x_previous =  np.mat ( ([0],
                                     [0],
                                     [0]) )

    def init_y (self):
        self.y =  np.mat ( ([0],
                            [0],
                            [0]) )
        self.y_previous =  np.mat ( ([0],
                                     [0],
                                     [0]) )

    def init_P (self):
        self.P =  np.mat ( ([1000.0, 0, 0],
                            [0, 1000.0, 0],
                            [0, 0, 1000.0]) )
        self.P_previous =  np.mat ( ([1000.0, 0, 0],
                                     [0, 1000.0, 0],
                                     [0, 0, 1000.0]) )

    def init_w (self):
        #self.w = np.mat ( (self.Q[0,0], self.Q[1,1], self.Q[2,2]) )
        #self.w = self.w.transpose ()

        self.w = np.mat ( (0.0, 0.0 ,0.0 ) )
        self.w = self.w.transpose ()

    def init_v (self):
        #self.v = np.mat ( (self.R[0,0], self.R[1,1]) )
        #self.v = self.v.transpose ()
        
        self.v = np.mat ( (0.0, 0.0 ) )
        self.v = self.v.transpose ()

    def update_F (self):
        self.F = np.mat (( [1, self.dt, -self.dt],
                           [0,       1,        0],
                           [0,       0,        1] ))

    def compute (self, z, u):
        self.z = z
        self.u = u
    

        # Algorithm starts here
        self.x = (self.F * self.x)
        self.P = (self.F * self.P * self.F.transpose()) + self.Q
        self.y = self.z - (self.H * self.x)
        self.S = (self.H * self.P * self.H.transpose()) + self.R
        self.K = self.P * self.H.transpose() * np.linalg.inv (self.S)
        self.x = self.x + (self.K * self.y)
        self.P = (np.identity (3) - (self.K * self.H)) * self.P

        return self.x
