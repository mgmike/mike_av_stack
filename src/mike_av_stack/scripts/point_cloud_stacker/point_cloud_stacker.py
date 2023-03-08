#!/home/mike/anaconda3/envs/waymo/bin/python3

import rospy
import numpy as np
import ros_numpy
from sensor_msgs.msg import PointCloud2, PointField

class Combiner:
    def __init__(self):
        
        self.pcl_combined = None
        self.first_pcl_time = 0
        self.rotation_time = 4.0 / 20.0
        self.publisher = rospy.Publisher("/carla/ego_vehicle/lidar/lidar1/point_cloud_full", PointCloud2, queue_size=10)

    def callback(self, point_cloud):
        
        point_cloud_2d = np.array([np.array(x.tolist()) for x in ros_numpy.point_cloud2.pointcloud2_to_array(point_cloud)])

        # Combine pcls until a full 360 point cloud is created
        # The point cloud time could be different depending on the type of data. So just use its first measurement
        # Last_pcl_time will be a float of seconds
        pcl_time = point_cloud.header.stamp.secs + (point_cloud.header.stamp.nsecs / 1000000000)

        # This is the first new pcl
        if self.pcl_combined is None:
            self.first_pcl_time = pcl_time
            self.pcl_combined = point_cloud_2d
            rospy.loginfo('Starting new combined pcl')
            return
        # This is one of many combinations
        else:
            self.pcl_combined = np.append(self.pcl_combined, point_cloud_2d, axis=0)
            print('shape of combined', self.pcl_combined.shape, 'shape of new pcl', point_cloud_2d.shape)
            (h,w) = self.pcl_combined.shape
            rospy.loginfo('Combining pcl')

        if self.first_pcl_time + self.rotation_time < pcl_time :
            ros_dtype = PointField.FLOAT32
            dtype = np.float32
            itemsize = np.dtype(dtype).itemsize
            data = self.pcl_combined.flatten().astype(dtype).tobytes()

            point_cloud2 = PointCloud2()
            point_cloud2.header.stamp = rospy.Time.now()
            point_cloud2.header.frame_id = "ego_vehicle/lidar/lidar1/point_cloud_full"
            point_cloud2.height = 1
            point_cloud2.width = h
            fields = [PointField(name=n, offset=i*itemsize, datatype=ros_dtype, count=1) for i,n in enumerate(['x','y','z','intensity'])]
            point_cloud2.fields = fields
            point_cloud2.is_bigendian = False
            point_cloud2.is_dense = False
            point_cloud2.point_step = (itemsize * 4)
            point_cloud2.row_step = (point_cloud2.point_step * h)
            point_cloud2.data = data
            self.publisher.publish(point_cloud2)
            self.pcl_combined = None

def main():
    rospy.init_node("tools", anonymous=True)

    combiner = Combiner()

    rospy.Subscriber("/carla/ego_vehicle/lidar/lidar1/point_cloud", PointCloud2, combiner.callback)

    rospy.spin()

if __name__ == '__main__':
    main()