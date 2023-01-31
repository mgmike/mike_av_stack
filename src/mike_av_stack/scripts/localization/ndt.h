#include "scan_matching.h"
#include <pcl/registration/ndt.h>
#include <pcl_conversions/pcl_conversions.h>

class NDT : public Scan_Matching
{
public:
	Pose startingPose;
	int iterations;
    pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;

    NDT(PointCloudT::Ptr target, Pose startingPose, int iterations);
	void get_transform(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);
};