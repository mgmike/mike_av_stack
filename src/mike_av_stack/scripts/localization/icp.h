#include "scan_matching.h"
#include <mutex>
#include <vector>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/icp.h>
#include <pcl/console/time.h>   // TicToc
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>

class ICP : public Scan_Matching
{
public:
	Pose pose;
	int iterations;
	int dist = 2;
    double leafSize = 0.5;
    bool gnss_rdy = false;
    bool imu_rdy = false;
	std::mutex ps_mutex;
	std::mutex rt_mutex;
	std::mutex tm_mutex;
    std::mutex it_mutex;
    Eigen::Matrix4d initTransform;
	Eigen::Matrix4d transformation_matrix;
    ros::NodeHandle nh;
    ros::Subscriber gnss_sub;
    ros::Subscriber imu_sub;

    ICP(PointCloudT::Ptr target, Pose startingPose, int iterations, ros::NodeHandle n);
    void gnss_update(const sensor_msgs::NavSatFixConstPtr& gnss);
    void imu_update(const sensor_msgs::ImuConstPtr& imu);
	void get_transform(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);
};