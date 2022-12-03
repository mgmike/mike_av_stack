# ---------------------------------------------------------------------
# Project "Track 3D-Objects Over Time"
# Copyright (C) 2020, Dr. Antje Muntzinger / Dr. Andreas Haja.
#
# Purpose of this file : Kalman filter class
#
# You should have received a copy of the Udacity license together with this program.
#
# https://www.udacity.com/course/self-driving-car-engineer-nanodegree--nd013
# ----------------------------------------------------------------------
#

# imports
import numpy as np

# add project directory to python path to enable relative imports
import os
import sys

# from pyrsistent import T
PACKAGE_PARENT = '..'
SCRIPT_DIR = os.path.dirname(os.path.realpath(os.path.join(os.getcwd(), os.path.expanduser(__file__))))
sys.path.append(os.path.normpath(os.path.join(SCRIPT_DIR, PACKAGE_PARENT)))

import tracking_params as params 

class Filter:
    '''Kalman filter class'''
    def __init__(self):
        pass

    def F(self):
        ############
        # Step 1: implement and return system matrix F
        ############
        dt = params.dt
        return np.matrix([[1,0,0,dt,0,0],
                         [0,1,0,0,dt,0],
                         [0,0,1,0,0,dt],
                         [0,0,0,1,0,0],
                         [0,0,0,0,1,0],
                         [0,0,0,0,0,1]])
        
        ############
        # END student code
        ############ 

    def Q(self):
        ############
        # Step 1: implement and return process noise covariance Q
        ############
        dt = params.dt
        q = params.q
        q1 = dt * q
        q2 = (dt**2 * q) / 2
        q3 = (dt**3 * q) / 3
        return np.matrix([[q3,0,0,q2,0,0],
                         [0,q3,0,0,q2,0],
                         [0,0,q3,0,0,q2],
                         [q2,0,0,q1,0,0],
                         [0,q2,0,0,q1,0],
                         [0,0,q2,0,0,q1]])
        
        ############
        # END student code
        ############ 

    def predict(self, track):
        ############
        # Step 1: predict state x and estimation error covariance P to next timestep, save x and P in track
        ############

        F = self.F()
        x = F * track.x
        P = F * track.P * F.transpose() + self.Q()
        track.set_x(x)
        track.set_P(P)
        
        ############
        # END student code
        ############ 

    def update(self, track, meas):
        ############
        # Step 1: update state x and covariance P with associated measurement, save x and P in track
        ############
        
        x = track.x
        P = track.P

        # Get the projection matrix that projects the state space into the measurement space
        H = meas.sensor.get_H(x)

        H_t = H.transpose()
        K = P * H_t * np.linalg.inv(self.S(track, meas, H))
        I = np.identity(params.dim_state)
        track.set_x(x + K * self.gamma(track, meas)) # gamma is transformation of state estimation to measurement state
        track.set_P((I - K * H) * P)

        ############
        # END student code
        ############ 
        track.update_attributes(meas)
    
    def gamma(self, track, meas):
        ############
        # Step 1: calculate and return residual gamma
        ############

        # x = track.x[0:3] # 6x1
        # if meas.sensor.name == 'lidar':
        #     H = meas.sensor.get_H(x)
        #     return meas.z - H[0:3, 0:3] * x
        # elif meas.sensor.name == 'camera':
        #     return meas.z - meas.sensor.get_hx(x)
        
        # return 0

        H = meas.sensor.get_hx(track.x)
        gamma = meas.z - H
        return gamma
        
        ############
        # END student code
        ############ 

    def S(self, track, meas, H):
        ############
        # Step 1: calculate and return covariance of residual S
        ############

        H_t = H.transpose()
        return H * track.P * H_t + meas.R
        
        ############
        # END student code
        ############ 