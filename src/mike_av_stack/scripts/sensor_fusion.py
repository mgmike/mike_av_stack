#!/usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs.msg import Image, PointCloud2
from std_msgs.msg import Header
from vision_msgs.msg import BoundingBox3D, ObjectHypothesisWithPose, Detection3D
from geometry_msgs.msg import Pose, Point, Vector3, Quaternion
import objdet_pcl as pcl
import objdet_detect as odet
import tools.ros_conversions.transformations as transformations
import ros_numpy

class SensorFusion:
    def __init__(self, model, configs):
        self.verbose = True
        self.model = model
        self.configs = configs
        self.classes = ['']
        self.pub_detection = rospy.Publisher('/sensor_fusion/detection', Detection3D, queue_size=10)
        rospy.init_node("sensor_fusion", anonymous=True)

    def imgCallback(self, image):
        rospy.loginfo('Got an image')

    def get_point_cloud_2d(self, pointcloud):
        # Convert the data from a 1d list of uint8s to a 2d list
        field_len = len(pointcloud.data)

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

    def pclCallback(self, pointCloud):
        rospy.loginfo('Got pointcloud')
        point_cloud_2d = self.get_point_cloud_2d(pointCloud)
        bev = pcl.bev_from_pcl(point_cloud_2d, self.configs )
        detections = odet.detect_objects(bev, self.model, self.configs)

        for det in detections:
            d3d = Detection3D()
            header = Header()
            hyp = ObjectHypothesisWithPose()
            bbx = BoundingBox3D()
            pose = Pose()
            pt = Point()
            pt.x = det[1]
            pt.y = det[2]
            pt.z = det[3]
            q = transformations.euler_to_quaternion(0, 0, det[7])
            ori = Quaternion(q[0], q[1], q[2], q[3])
            pose.position = pt
            pose.orientation = ori
            size = Vector3()
            size.x = det[4]
            size.y = det[5]
            size.z = det[6]
            bbx.center = pose
            bbx.size = size
            d3d.header = header
            d3d.results = [hyp]
            d3d.bbox = bbx
            self.pub_detection.publish(d3d)
            
        

def main():
    configs_det = odet.load_configs(model_name='fpn_resnet')
    model_det = odet.create_model(configs_det)
    sf = SensorFusion(model=model_det, configs=configs_det)

    rospy.loginfo('Setting up publishers')

    rospy.loginfo('Setting up listeners')

    # rospy.Subscriber("/carla/ego_vehicle/camera/rgb/front/image_color", Image, sf.imgCallback)
    rospy.Subscriber("/carla/ego_vehicle/lidar/lidar1/point_cloud", PointCloud2, sf.pclCallback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

    
if __name__ == '__main__':
    main()