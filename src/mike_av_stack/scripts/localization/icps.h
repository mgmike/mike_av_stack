#include "scan_matching.h"
#include <vector>
#include <Eigen/SVD>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
// for pcl::transformPointCloud
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

class ICPS : public Scan_Matching
{
public:
	Pose startingPose;
	int iterations;
	int dist = 2;
	Eigen::Matrix4d transformation_matrix_old;
	Eigen::Matrix4d transformation_matrix;
  	pcl::KdTreeFLANN<PointT> kdtree;

    ICPS(PointCloudT::Ptr target, Pose startingPose, int iterations, int dist);
	void get_transform(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);
};