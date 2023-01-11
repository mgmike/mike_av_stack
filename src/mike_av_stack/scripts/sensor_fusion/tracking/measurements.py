# ---------------------------------------------------------------------
# Project "Track 3D-Objects Over Time"
# Copyright (C) 2020, Dr. Antje Muntzinger / Dr. Andreas Haja.
#
# Purpose of this file : Classes for sensor and measurement 
#
# You should have received a copy of the Udacity license together with this program.
#
# https://www.udacity.com/course/self-driving-car-engineer-nanodegree--nd013
# ----------------------------------------------------------------------
#

# imports
import numpy as np
import rospy


from std_msgs.msg import Header
from sensor_msgs.msg import Image, PointCloud2
from vision_msgs.msg import BoundingBox3D, ObjectHypothesisWithPose, Detection3D, Detection3DArray
from geometry_msgs.msg import Pose, Point, Vector3, Quaternion
import detection.objdet_pcl as pcl
import detection.objdet_detect as odet
import numpy as np
import time

import os
import sys
dir_tracking = os.path.dirname(os.path.realpath(__file__))
dir_sf = os.path.dirname(dir_tracking)
dir_scripts = os.path.dirname(dir_sf)
sys.path.append(dir_scripts)
import tools.ros_conversions.transformations as transformations
import ros_numpy


class Sensor:
    '''Sensor class including measurement matrix'''
    def __init__(self, name, configs, configs_track_management = None):
        self.verbose = True
        self.configs = configs
        self.name = name

        self.frame_id = 0

        self.pub_detection = rospy.Publisher('/sensor_fusion/detection/' + self.configs.id, Detection3DArray, queue_size=10)

    def callback(self):
        """ Override this for subscriber """
    
    def in_fov(self, x):
        """ check if an object x can be seen by this sensor """
        
             
    def get_hx(self, x):    
        """ calculate nonlinear measurement expectation value h(x) """

        
    def get_H(self, x, params):
        """ calculate Jacobian H at current x from h(x) """

class Lidar(Sensor):
    def __init__(self, name, configs, configs_track_management=None):
        super().__init__(name, configs, configs_track_management)

        self.configs.update(odet.load_configs())
        self.model = odet.create_model(self.configs)

        # Set up transforms
        self.sens_to_veh = np.matrix(np.identity((4))) # transformation sensor to vehicle coordinates equals identity matrix because lidar detections are already in vehicle coordinates
        print(type(self.sens_to_veh))
        self.veh_to_sens = np.linalg.inv(self.sens_to_veh) # transformation vehicle to sensor coordinates

        # Set up ros subscriber
        rospy.Subscriber(self.configs.base_topic, PointCloud2, self.callback)
        self.fov = [-np.pi/2, np.pi/2] # angle of field of view in radians
        self.dim_meas = 3

        self.configs.rotation_time = 4.0 / self.configs.rotation_frequency
        self.first_pcl_time = 0
        self.pcl_combined = None
        
    # callback will have to be async
    # Will need to add a mutex to pcl_combined and last_pcl_time
    def callback(self, point_cloud):
        if self.verbose:
            rospy.loginfo('Got pointcloud')

        point_cloud_2d = self.get_point_cloud_2d(point_cloud)


        # Combine pcls until a full 360 point cloud is created
        # The point cloud time could be different depending on the type of data. So just use its first measurement
        # Last_pcl_time will be a float of seconds
        pcl_time = point_cloud.header.stamp.secs + (point_cloud.header.stamp.nsecs / 1000000000)

        info_string = 'first pcl time: ' + str(self.first_pcl_time) + ' rotation time: ' + str(self.configs.rotation_time) + ' pcl time: ' + str(pcl_time)
        rospy.loginfo(info_string)

        # This is the first new pcl
        if self.pcl_combined is None:
            self.first_pcl_time = pcl_time
            self.pcl_combined = point_cloud_2d
            rospy.loginfo('Starting new combined pcl')
            return
        # This is one of many combinations
        else:
            print('shape of combined', self.pcl_combined.shape, 'shape of new pcl', point_cloud_2d.shape)
            self.pcl_combined = np.append(self.pcl_combined, point_cloud_2d, axis=0)
            rospy.loginfo('Combining pcl')


        if self.first_pcl_time + self.configs.rotation_time < pcl_time :

            rospy.loginfo('Getting detection of full pcl')
            bev = pcl.bev_from_pcl(self.pcl_combined, self.configs)
            detections = odet.detect_objects(bev, self.model, self.configs)

            if self.verbose:
                print(len(detections))

            dets = []
            for det in detections:
                d3d = Detection3D()
                q = transformations.euler_to_quaternion(0, 0, det[7])
                ori = Quaternion(q[0], q[1], q[2], q[3])
                d3d.bbox.center.orientation = ori
                d3d.bbox.center.position.x = det[1]
                d3d.bbox.center.position.y = det[2]
                d3d.bbox.center.position.z = det[3]
                d3d.bbox.size.x = det[4]
                d3d.bbox.size.y = det[5]
                d3d.bbox.size.z = det[6]
                dets.append(d3d)

            time_now = time.time_ns()
            header = Header()
            self.frame_id += 1
            header.frame_id = self.frame_id 
            header.stamp.secs = int(time_now / 10e9)
            header.stamp.nsecs = time_now - (header.stamp.secs * 10e9)
            detection3DArray = Detection3DArray()
            detection3DArray.header = Header() 
            detection3DArray.header
            detection3DArray.detections = dets
            self.pub_detection.publish(detection3DArray)
            self.pcl_combined = None

    def get_point_cloud_2d(self, pointcloud):
        # Convert the data from a 1d list of uint8s to a 2d list
        field_len = len(pointcloud.data)

        # The result of this is <vector<vector>> where [[]]
        point_cloud_2d = np.array([np.array(x.tolist()) for x in ros_numpy.point_cloud2.pointcloud2_to_array(pointcloud)])
    
        if self.verbose:
            print("Shape of pc2d: ", point_cloud_2d.shape, " First element: ", type(point_cloud_2d[0]), point_cloud_2d[0])
            print("First og: ", pointcloud.data[0], ", ", pointcloud.data[1], ", ", pointcloud.data[2], ", ", pointcloud.data[3])
            print("height: %d, width: %d, length of data: %d" % (pointcloud.height, pointcloud.width, field_len))
            for field in pointcloud.fields:
                print("\tname: ", field.name, "offset: ", field.offset, "datatype: ", field.datatype, "count: ", field.count)

        # TODO: Will need to transform to vehicle coordinate system

        # perform coordinate conversion
        # xyz_sensor = np.stack([x,y,z,np.ones_like(z)])
        # xyz_vehicle = np.einsum('ij,jkl->ikl', extrinsic, xyz_sensor)
        # xyz_vehicle = xyz_vehicle.transpose(1,2,0)

        # transform 3d points into vehicle coordinate system
        # pcl = xyz_vehicle[ri_range > 0,:3]
        # pcd = o3d.geometry.PointCloud()
        # pcd.points = o3d.utility.Vector3dVector(pcl)
        # o3d.visualization.draw_geometries([pcd])

        return point_cloud_2d

    def in_fov(self, x):
        x_s = x[0:4]
        x_s[3] = 1
        x_v = self.veh_to_sens * x_s
        if x_v[0] != 0:
            alpha = np.arctan(x_v[1] / x_v[0])
            if alpha > self.fov[0] and alpha < self.fov[1]:
                return True

        return False

    def get_hx(self, x):
        pos_veh = np.ones((4, 1)) # homogeneous coordinates
        pos_veh[0:3] = x[0:3] 
        pos_sens = self.veh_to_sens*pos_veh # transform from vehicle to lidar coordinates
        return pos_sens[0:3]


    def get_H(self, x, params):
        H = np.matrix(np.zeros((self.dim_meas, params.dim_state)))
        R = self.veh_to_sens[0:3, 0:3] # rotation
        T = self.veh_to_sens[0:3, 3] # translation
        H[0:3, 0:3] = R
        return H
    

class Camera(Sensor):
    def __init__(self, name, configs, configs_track_management=None):
        super().__init__(name, configs, configs_track_management)
        rospy.Subscriber(self.configs.base_topic, Image, self.callback)
        self.fov = [-0.35, 0.35] # angle of field of view in radians, inaccurate boundary region was removed

        self.dim_meas = 2
        if 'calib' not in self.configs :
            rospy.loginfo('No calibration settings in the sensors.json file. Consider adding them')
            return

        calib = self.configs.calib

        self.sens_to_veh = np.matrix(calib.extrinsic.transform).reshape(4,4) # transformation sensor to vehicle coordinates
        self.f_i = calib.intrinsic[0] # focal length i-coordinate
        self.f_j = calib.intrinsic[1] # focal length j-coordinate
        self.c_i = calib.intrinsic[2] # principal point i-coordinate
        self.c_j = calib.intrinsic[3] # principal point j-coordinate

        self.veh_to_sens = np.linalg.inv(self.sens_to_veh) # transformation vehicle to sensor coordinates

        
    def callback(self, image):
        rospy.loginfo('Got an image')
    
    def get_hx(self, x):
        ############
        # Step 4: implement nonlinear camera measurement function h:
        # - transform position estimate from vehicle to camera coordinates
        # - project from camera to image coordinates
        # - make sure to not divide by zero, raise an error if needed
        # - return h(x)
        ############

        pos_veh = np.ones((4,1))
        pos_veh[0:3] = x[0:3]
        pos_sens = self.veh_to_sens * pos_veh

        hx = np.zeros((self.dim_meas,1))
        px, py, pz, _ = pos_sens

        if px == 0:
            raise NameError('Jacobain is not defined for px=0!')
        else:
            hx[0] = self.c_i - (self.f_i * py) / px
            hx[1] = self.c_j - (self.f_j * pz) / px
        
        return hx  

    def get_H(self, x, params):
        H = np.matrix(np.zeros((self.dim_meas, params.dim_state)))
        R = self.veh_to_sens[0:3, 0:3] # rotation
        T = self.veh_to_sens[0:3, 3] # translation
        # check and print error message if dividing by zero
        if R[0,0]*x[0] + R[0,1]*x[1] + R[0,2]*x[2] + T[0] == 0: 
            raise NameError('Jacobian not defined for this x!')
        else:
            H[0,0] = self.f_i * (-R[1,0] / (R[0,0]*x[0] + R[0,1]*x[1] + R[0,2]*x[2] + T[0])
                                + R[0,0] * (R[1,0]*x[0] + R[1,1]*x[1] + R[1,2]*x[2] + T[1]) \
                                    / ((R[0,0]*x[0] + R[0,1]*x[1] + R[0,2]*x[2] + T[0])**2))
            H[1,0] = self.f_j * (-R[2,0] / (R[0,0]*x[0] + R[0,1]*x[1] + R[0,2]*x[2] + T[0])
                                + R[0,0] * (R[2,0]*x[0] + R[2,1]*x[1] + R[2,2]*x[2] + T[2]) \
                                    / ((R[0,0]*x[0] + R[0,1]*x[1] + R[0,2]*x[2] + T[0])**2))
            H[0,1] = self.f_i * (-R[1,1] / (R[0,0]*x[0] + R[0,1]*x[1] + R[0,2]*x[2] + T[0])
                                + R[0,1] * (R[1,0]*x[0] + R[1,1]*x[1] + R[1,2]*x[2] + T[1]) \
                                    / ((R[0,0]*x[0] + R[0,1]*x[1] + R[0,2]*x[2] + T[0])**2))
            H[1,1] = self.f_j * (-R[2,1] / (R[0,0]*x[0] + R[0,1]*x[1] + R[0,2]*x[2] + T[0])
                                + R[0,1] * (R[2,0]*x[0] + R[2,1]*x[1] + R[2,2]*x[2] + T[2]) \
                                    / ((R[0,0]*x[0] + R[0,1]*x[1] + R[0,2]*x[2] + T[0])**2))
            H[0,2] = self.f_i * (-R[1,2] / (R[0,0]*x[0] + R[0,1]*x[1] + R[0,2]*x[2] + T[0])
                                + R[0,2] * (R[1,0]*x[0] + R[1,1]*x[1] + R[1,2]*x[2] + T[1]) \
                                    / ((R[0,0]*x[0] + R[0,1]*x[1] + R[0,2]*x[2] + T[0])**2))
            H[1,2] = self.f_j * (-R[2,2] / (R[0,0]*x[0] + R[0,1]*x[1] + R[0,2]*x[2] + T[0])
                                + R[0,2] * (R[2,0]*x[0] + R[2,1]*x[1] + R[2,2]*x[2] + T[2]) \
                                    / ((R[0,0]*x[0] + R[0,1]*x[1] + R[0,2]*x[2] + T[0])**2))
        return H   
        
class Measurement:
    '''Measurement class including measurement values, covariance, timestamp, sensor'''
    def __init__(self, num_frame, detection, sensor, params):
        # create measurement object
        self.t = (num_frame - 1) * params.dt # time
        self.sensor = sensor # sensor that generated this measurement
        
        if sensor.name == 'lidar':
            sigma_lidar_x = params.sigma_lidar_x # load params
            sigma_lidar_y = params.sigma_lidar_y
            sigma_lidar_z = params.sigma_lidar_z
            self.z = np.zeros((sensor.dim_meas,1)) # measurement vector
            self.z[0] = detection.bbox.center.position.x
            self.z[1] = detection.bbox.center.position.y
            self.z[2] = detection.bbox.center.position.z
            self.R = np.matrix([[sigma_lidar_x**2, 0, 0], # measurement noise covariance matrix
                                [0, sigma_lidar_y**2, 0], 
                                [0, 0, sigma_lidar_z**2]])
            
            self.width = detection.bbox.size.x
            self.length = detection.bbox.size.y
            self.height = detection.bbox.size.z
            self.yaw = transformations.quaternion_to_euiler(detection.bbox.center.orientation)
        elif sensor.name == 'camera':
            
            ############
            # Step 4: initialize camera measurement including z and R 
            ############

            sigma_cam_i = params.sigma_cam_i # load params
            sigma_cam_j = params.sigma_cam_j
            self.z = np.zeros((sensor.dim_meas, 1))
            self.z[0] = detection.bbox.center.x
            self.z[1] = detection.bbox.center.y
            self.length = detection.bbox.size_x
            self.width = detection.bbox.size_y
            self.R = np.matrix([[sigma_cam_i**2, 0], # measurement noise covariance matrix
                                [0, sigma_cam_j**2]])
            