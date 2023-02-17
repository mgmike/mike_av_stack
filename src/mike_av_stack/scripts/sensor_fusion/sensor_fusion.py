#!/home/mike/anaconda3/envs/waymo/bin/python3

import rospy
import os
import json
from easydict import EasyDict as edict

from sensor_msgs.msg import Image, PointCloud2
from vision_msgs.msg import Detection3DArray
# from tracking.trackmanagement import Trackmanagement
from tracking.measurements import Sensor, Lidar, Camera
from tracking.trackmanagement import Trackmanagement
    

def get_sensor(sensor, trackmanager):
    name = sensor.type.split('.')[1]
    name = sensor.type.split('.')[2] if 'other' in name else name
    if name == 'lidar':
        return Lidar(name, sensor, trackmanager)
    elif name == 'camera':
        return Camera(name, sensor, trackmanager)

def main():
    
    curr_path = os.path.dirname(os.path.realpath(__file__))
    parent_path = os.path.abspath(os.path.join(curr_path, os.pardir))  
    
    sensors_j = edict()
    # Create edict json object of all the sensors in sensors.json
    with open(os.path.join(curr_path, 'configs', 'sensors.json')) as j_object:
        sensors_j.update(json.load(j_object))

    # print(sensors_j.sensors)

    trackmanager = Trackmanagement()

    rospy.init_node("sensor_fusion", anonymous=True)

    # Create list of Sensors
    sensors = {sensor.id : get_sensor(sensor, trackmanager) for sensor in sensors_j.sensors}
    


    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

    
if __name__ == '__main__':
    main()