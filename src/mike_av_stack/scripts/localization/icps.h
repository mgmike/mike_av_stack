#include "scan_matching.h"

class ICPS : public Scan_Matching
{
private:
	PointCloudT::Ptr target;
	Pose startingPose;
	int iterations;
public:
    ICPS(PointCloudT::Ptr target&, Pose startingPose, int iterations);
	void set_map(PointCloudT::Ptr target);
	Eigen::Matrix4d get_transform(const sensor_msgs::PointCloud2ConstPtr& source);
}