#include "scan_matching.h"
#include <pcl/registration/ndt.h>
#include <pcl_conversions/pcl_conversions.h>

class NDT : public Scan_Matching
{
public:
	Pose startingPose;
	int iterations;
    pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;

    NDT(PointCloudT::Ptr target, std::string topic, Pose startingPose, int iterations);
	void get_transform(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg);
};