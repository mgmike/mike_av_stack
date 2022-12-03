# ---------------------------------------------------------------------
# Project "Track 3D-Objects Over Time"
# Copyright (C) 2020, Dr. Antje Muntzinger / Dr. Andreas Haja.
#
# Purpose of this file : Classes for track and track management
#
# You should have received a copy of the Udacity license together with this program.
#
# https://www.udacity.com/course/self-driving-car-engineer-nanodegree--nd013
# ----------------------------------------------------------------------
#

# imports
import numpy as np
import collections
import rospy
from vision_msgs.msg import BoundingBox3D, ObjectHypothesisWithPose, Detection3D

# add project directory to python path to enable relative imports
import os
import sys
PACKAGE_PARENT = '..'
SCRIPT_DIR = os.path.dirname(os.path.realpath(os.path.join(os.getcwd(), os.path.expanduser(__file__))))
sys.path.append(os.path.normpath(os.path.join(SCRIPT_DIR, PACKAGE_PARENT)))
import tracking_params as params 

import tracking.tracking_params as params
from tracking.filter import Filter
from tracking.association import Association
from tracking.trackmanagement import Trackmanagement
from tracking.measurements import Sensor, Measurement

class Track:
    '''Track class with state, covariance, id, score'''
    def __init__(self, meas, id):
        print('creating track no.', id)
        M_rot = meas.sensor.sens_to_veh[0:3, 0:3] # rotation matrix from sensor to vehicle coordinates
        
        ############
        # Step 2: initialization:
        # - replace fixed track initialization values by initialization of x and P based on 
        # unassigned measurement transformed from sensor to vehicle coordinates
        # - initialize track state and track score with appropriate values
        ############

        self.x = np.zeros((6,1))
        self.P = np.zeros((6,6))
        z = np.zeros((4,1))
        z[3] = 1
        z[0:3] = meas.z
        T = meas.sensor.sens_to_veh
        R = meas.R
        M_rot = T[0:3, 0:3]

        # Compute position estimation and error covariance
        self.x[0:3] = (T * z)[0:3]
        self.P[0:3, 0:3] = M_rot * R * M_rot.transpose()

        # Compute velocity estimation error covariance
        P_vel = np.matrix([[params.sigma_p44**2, 0, 0],
                           [0, params.sigma_p55**2, 0],
                           [0, 0, params.sigma_p66**2]])
        self.P[3:6, 3:6] = P_vel

        # self.x = np.matrix([[49.53980697],
        #                 [ 3.41006279],
        #                 [ 0.91790581],
        #                 [ 0.        ],
        #                 [ 0.        ],
        #                 [ 0.        ]])
        # self.P = np.matrix([[9.0e-02, 0.0e+00, 0.0e+00, 0.0e+00, 0.0e+00, 0.0e+00],
        #                 [0.0e+00, 9.0e-02, 0.0e+00, 0.0e+00, 0.0e+00, 0.0e+00],
        #                 [0.0e+00, 0.0e+00, 6.4e-03, 0.0e+00, 0.0e+00, 0.0e+00],
        #                 [0.0e+00, 0.0e+00, 0.0e+00, 2.5e+03, 0.0e+00, 0.0e+00],
        #                 [0.0e+00, 0.0e+00, 0.0e+00, 0.0e+00, 2.5e+03, 0.0e+00],
        #                 [0.0e+00, 0.0e+00, 0.0e+00, 0.0e+00, 0.0e+00, 2.5e+01]])

        self.state = 'initialized'
        self.score = 1.0 / params.window
        
        ############
        # END student code
        ############ 
               
        # other track attributes
        self.id = id
        self.width = meas.width
        self.length = meas.length
        self.height = meas.height
        self.yaw =  np.arccos(M_rot[0,0]*np.cos(meas.yaw) + M_rot[0,1]*np.sin(meas.yaw)) # transform rotation from sensor to vehicle coordinates
        self.t = meas.t

    def set_x(self, x):
        self.x = x
        
    def set_P(self, P):
        self.P = P  
        
    def set_t(self, t):
        self.t = t  
        
    def update_attributes(self, meas):
        # use exponential sliding average to estimate dimensions and orientation
        if meas.sensor.name == 'lidar':
            c = params.weight_dim
            self.width = c*meas.width + (1 - c)*self.width
            self.length = c*meas.length + (1 - c)*self.length
            self.height = c*meas.height + (1 - c)*self.height
            M_rot = meas.sensor.sens_to_veh
            self.yaw = np.arccos(M_rot[0,0]*np.cos(meas.yaw) + M_rot[0,1]*np.sin(meas.yaw)) # transform rotation from sensor to vehicle coordinates
        
        
###################        

class Trackmanagement:
    '''Track manager with logic for initializing and deleting objects'''
    def __init__(self, sensors):
        self.sensors = sensors
        self.N = 0 # current number of tracks
        self.track_list = []
        self.last_id = -1
        self.result_list = []
        
        rospy.loginfo('Setting up listeners')
        # rospy.Subscriber("/carla/ego_vehicle/camera/rgb/front/image_color", Image, sf.imgCallback)
        rospy.Subscriber("/sensor_fusion/detection", Detection3D, self.detection_callback)

        filter = Filter()
        association = Association()
        self.meas_list = []

    def detection_callback(self, detections):
        # predict
        for detection in detections:
            meas = Measurement()
            self.meas_list.append(meas)
        
    def manage_tracks(self, unassigned_tracks, unassigned_meas, meas_list):  
        ############
        # Step 2: implement track management:
        # - decrease the track score for unassigned tracks
        # - delete tracks if the score is too low or P is too big (check params.py for parameters that might be helpful, but
        # feel free to define your own parameters)
        ############

        # Seems like the value of each element in the unassigned_tracks list will be an index of the track in track_list
        
        # decrease score for unassigned tracks
        for i in unassigned_tracks:
            u = self.track_list[i]
            if meas_list:
                if meas_list[0].sensor.in_fov(u.x):
                    u.score -= 1.0 / params.window
            # else: 
            #     u.score -= 1.0 / params.window
            if u.score <= 0.0:
                u.score = 0.0

        # delete old tracks   
        for track in self.track_list:
            if ((track.state in ['confirmed'] and track.score < params.delete_threshold) or
                    ((track.P[0, 0] > params.max_P or track.P[1, 1] > params.max_P)) or
                    track.score < 0.05):
                self.delete_track(track)

        ############
        # END student code
        ############ 
            
        # initialize new track with unassigned measurement
        for j in unassigned_meas: 
            if meas_list[j].sensor.name == 'lidar': # only initialize with lidar measurements
                self.init_track(meas_list[j])
            
    def addTrackToList(self, track):
        self.track_list.append(track)
        self.N += 1
        self.last_id = track.id

    def init_track(self, meas):
        track = Track(meas, self.last_id + 1)
        self.addTrackToList(track)

    def delete_track(self, track):
        print('deleting track no.', track.id)
        self.track_list.remove(track)
        
    def handle_updated_track(self, track):      
        ############
        # Step 2: implement track management for updated tracks:
        # - increase track score
        # - set track state to 'tentative' or 'confirmed'
        ############

        track.score += 1 / params.window
        track.score = min(1.0, track.score)

        if track.score > params.confirmed_threshold:
            track.state = 'confirmed'
        else:
            track.state = 'tentative'

        
        ############
        # END student code
        ############ 