#include "scan_matching.h"
#include <pcl/registration/ndt.h>

class NDT : public Scan_Matching
{
private:
	PointCloudT::Ptr target;
	Pose startingPose;
	int iterations;
    pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
public:
    NDT(PointCloudT::Ptr target, Pose startingPose, int iterations);
	void set_map(PointCloudT::Ptr target);
	Eigen::Matrix4d get_transform(const sensor_msgs::PointCloud2ConstPtr& source);
}