#!/home/mike/anaconda3/envs/waymo/bin/python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
from vision_msgs.msg import Detection3DArray

class Combiner(Node):
    def __init__(self):
        super().__init__('point_cloud_stacker')
        self.pcl_combined = None
        self.first_pcl_time = 0
        self.rotation_time = 4.0 / 20.0

        self.get_logger().info('Starting point_cloud_stacker node')


        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=5
        )
        self.subscriber_combiner = self.create_subscription(
            msg_type=PointCloud2,
            topic='/carla/ego_vehicle/lidar1', 
            callback=self.callback,
            qos_profile=qos_profile
            )
        self.subscriber_combiner

        self.publisher = self.create_publisher(
            PointCloud2, 
            "/carla/ego_vehicle/lidar1/point_cloud_full",
            10
            )


    def callback(self, point_cloud):
        
        point_cloud_2d = np.array([np.array(x.tolist()) for x in pointcloud2_to_array(point_cloud)])

        # Combine pcls until a full 360 point cloud is created
        # The point cloud time could be different depending on the type of data. So just use its first measurement
        # Last_pcl_time will be a float of seconds
        pcl_time = point_cloud.header.stamp.sec + (point_cloud.header.stamp.nanosec / 1000000000)

        # This is the first new pcl
        if self.pcl_combined is None:
            self.first_pcl_time = pcl_time
            self.pcl_combined = point_cloud_2d
            self.get_logger().debug('Starting new combined pcl')
            return
        # This is one of many combinations
        else:
            self.pcl_combined = np.append(self.pcl_combined, point_cloud_2d, axis=0)
            self.get_logger().debug(f'shape of combined {self.pcl_combined.shape} shape of new pcl {point_cloud_2d.shape}')
            (h,w) = self.pcl_combined.shape
            self.get_logger().debug('Combining pcl')

        if self.first_pcl_time + self.rotation_time < pcl_time :
            ros_dtype = PointField.FLOAT32
            dtype = np.float32
            itemsize = np.dtype(dtype).itemsize
            data = self.pcl_combined.flatten().astype(dtype).tobytes()

            point_cloud2 = PointCloud2()
            point_cloud2.header.stamp = self.get_clock().now().to_msg()
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


# I took this from Box-Robotics ros2_numpy because it wont compile as an ament package

# prefix to the names of dummy fields we add to get byte alignment
# correct. this needs to not clash with any actual field names
DUMMY_FIELD_PREFIX = '__'

# mappings between PointField types and numpy types
type_mappings = [(PointField.INT8, np.dtype('int8')),
                 (PointField.UINT8, np.dtype('uint8')),
                 (PointField.INT16, np.dtype('int16')),
                 (PointField.UINT16, np.dtype('uint16')),
                 (PointField.INT32, np.dtype('int32')),
                 (PointField.UINT32, np.dtype('uint32')),
                 (PointField.FLOAT32, np.dtype('float32')),
                 (PointField.FLOAT64, np.dtype('float64'))]
pftype_to_nptype = dict(type_mappings)

def pointcloud2_to_array(cloud_msg, squeeze=True):
    ''' Converts a rospy PointCloud2 message to a numpy recordarray

    Reshapes the returned array to have shape (height, width), even if the
    height is 1.

    The reason for using np.frombuffer rather than struct.unpack is
    speed... especially for large point clouds, this will be <much> faster.
    '''
    # construct a numpy record type equivalent to the point type of this cloud
    dtype_list = fields_to_dtype(cloud_msg.fields, cloud_msg.point_step)

    # parse the cloud into an array
    cloud_arr = np.frombuffer(cloud_msg.data, dtype_list)

    # remove the dummy fields that were added
    cloud_arr = cloud_arr[
        [fname for fname, _type in dtype_list if not (
            fname[:len(DUMMY_FIELD_PREFIX)] == DUMMY_FIELD_PREFIX)]]

    if squeeze and cloud_msg.height == 1:
        return np.reshape(cloud_arr, (cloud_msg.width,))
    else:
        return np.reshape(cloud_arr, (cloud_msg.height, cloud_msg.width))

def fields_to_dtype(fields, point_step):
    '''Convert a list of PointFields to a numpy record datatype.
    '''
    offset = 0
    np_dtype_list = []
    for f in fields:
        while offset < f.offset:
            # might be extra padding between fields
            np_dtype_list.append(
                ('%s%d' % (DUMMY_FIELD_PREFIX, offset), np.uint8))
            offset += 1

        dtype = pftype_to_nptype[f.datatype]
        if f.count != 1:
            dtype = np.dtype((dtype, f.count))

        np_dtype_list.append((f.name, dtype))
        offset += pftype_to_nptype[f.datatype].itemsize * f.count

    # might be extra padding between points
    while offset < point_step:
        np_dtype_list.append(('%s%d' % (DUMMY_FIELD_PREFIX, offset), np.uint8))
        offset += 1

    return np_dtype_list

def main(args=None):
    rclpy.init(args=args)
    combiner = Combiner()
    rclpy.spin(combiner, None)
    rclpy.shutdown()
    return

if __name__ == '__main__':
    main()