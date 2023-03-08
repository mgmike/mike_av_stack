#include "helper.h"
#include <ros/console.h>
#include <Eigen/Geometry>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/visualization/pcl_visualizer.h>

#ifndef SCANMATCHING_H
#define SCANMATCHING_H

class Scan_Matching
{
protected:
	PointCloudT::Ptr target;
	bool viz = false;
	pcl::visualization::PCLVisualizer::Ptr viewer;

public:
	Scan_Matching(PointCloudT::Ptr target);
	virtual void set_map(PointCloudT::Ptr target);
	virtual void get_transform(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);
	virtual void enable_viz();
};

#endif