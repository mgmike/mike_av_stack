#include "helper.h"
#include <Eigen/Geometry>
#include <sensor_msgs/PointCloud2.h>

#ifndef SCANMATCHING_H
#define SCANMATCHING_H

class Scan_Matching
{
public:
	PointCloudT::Ptr target;
	
	virtual void set_map(PointCloudT::Ptr target);
	virtual void get_transform(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);
};

#endif