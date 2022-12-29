#!/home/mike/anaconda3/envs/waymo/bin/python3

import rospy
import os
import json
from easydict import EasyDict as edict

from sensor_msgs.msg import Image, PointCloud2
from vision_msgs.msg import Detection3DArray
# from tracking.trackmanagement import Trackmanagement
from tracking.measurements import Sensor


class SensorFusion:
    def __init__(self, sensors, verbose=False):
        self.verbose = verbose
        self.classes = ['']
        self.frame_id = 0
        rospy.init_node("sensor_fusion", anonymous=True)

        for sensor in sensors:
            if "base_topic" in sensor.configs:
                base_topic = sensor.configs.base_topic
                if sensor.name == 'lidar':
                    rospy.Subscriber(base_topic, PointCloud2, sensor.pclCallback)
                elif sensor.name == 'camera':
                    rospy.Subscriber(base_topic, Image, sensor.imgCallback)

        

def main():
    
    curr_path = os.path.dirname(os.path.realpath(__file__))
    parent_path = os.path.abspath(os.path.join(curr_path, os.pardir))  
    
    sensors_j = edict()

    with open(os.path.join(curr_path, 'configs', 'sensors.json')) as j_object:
        sensors_j.update(json.load(j_object))

    print(sensors_j.sensors)

    sensors = {sensor.id : Sensor(sensor) for sensor in sensors_j.sensors}
    
    sf = SensorFusion(sensors)
    # tm = Trackmanagement(sensors)


    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

    
if __name__ == '__main__':
    main()