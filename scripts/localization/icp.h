#include "scan_matching.h"
#include <mutex>
#include <vector>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/icp.h>
#include <pcl/console/time.h>   // TicToc
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/imu.hpp"

class ICP : public Scan_Matching
{
public:
	Pose pose;
	int iterations;
	int dist = 2;
    double leafSize = 0.5;
    bool gnss_rdy = false;
    bool imu_rdy = false;
	std::mutex *ps_mutex;
	std::mutex *rt_mutex;
	std::mutex *tm_mutex;
    std::mutex *it_mutex;
    Eigen::Matrix4d initTransform;
	Eigen::Matrix4d transformation_matrix;
	rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gnss_sub;
	rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;

    ICP(PointCloudT::Ptr target, std::string topic, Pose startingPose, int iterations);
    void gnss_update(const sensor_msgs::msg::NavSatFix::SharedPtr gnss);
    void imu_update(const sensor_msgs::msg::Imu::SharedPtr imu);
	void get_transform(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg);
};