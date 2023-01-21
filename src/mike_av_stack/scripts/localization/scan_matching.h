#include "helper.h"
#include <Eigen/Geometry>
#include <sensor_msgs/PointCloud2.h>

struct Pair{

	Point p1;
	Point p2;

	Pair(Point setP1, Point setP2)
		: p1(setP1), p2(setP2){}
};

class Scan_Matching
{
private:
	PointCloudT::Ptr target;
public:
	Scan_Matching(PointCloudT::Ptr target&);
	void set_map(PointCloudT::Ptr target&);
	Eigen::Matrix4d get_transform(const sensor_msgs::PointCloud2ConstPtr& source);
}