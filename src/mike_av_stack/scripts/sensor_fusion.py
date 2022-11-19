#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
import objdet_pcl as pcl

def imgCallback(image):
    rospy.loginfo('Got an image')

def pclCallback(pointCloud):
    rospy.loginfo('Got pointcloud')
    bev = pcl.bev_from_pcl(pointCloud)

    

def listener():

    rospy.loginfo('Setting up listeners')

    rospy.init_node('mike_av_stack_sensor_fusion', anonymous=True)
    rospy.Subscriber("/carla/ego_vehicle/camera/rgb/front/image_color", Image, imgCallback)
    rospy.Subscriber("/carla/ego_vehicle/lidar/lidar1/point_cloud", PointCloud2, pclCallback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
    
if __name__ == '__main__':
    listener()