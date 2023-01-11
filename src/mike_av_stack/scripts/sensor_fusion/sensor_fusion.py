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


class SensorFusion:
    def __init__(self, sensors, verbose=False):
        self.verbose = verbose
        self.classes = ['']
        self.frame_id = 0
        self.sensors = sensors
        rospy.init_node("sensor_fusion", anonymous=True)

    

def get_sensor(sensor):
    name = sensor.type.split('.')[1]
    name = sensor.type.split('.')[2] if 'other' in name else name
    if name == 'lidar':
        return Lidar(name, sensor)
    elif name == 'camera':
        return Camera(name, sensor)

def main():
    
    curr_path = os.path.dirname(os.path.realpath(__file__))
    parent_path = os.path.abspath(os.path.join(curr_path, os.pardir))  
    
    sensors_j = edict()
    # Create edict json object of all the sensors in sensors.json
    with open(os.path.join(curr_path, 'configs', 'sensors.json')) as j_object:
        sensors_j.update(json.load(j_object))

    # print(sensors_j.sensors)

    # Create list of Sensors
    sensors = {sensor.id : get_sensor(sensor) for sensor in sensors_j.sensors}
    
    sf = SensorFusion(sensors)
    tm = Trackmanagement(sensors)


    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

    
if __name__ == '__main__':
    main()