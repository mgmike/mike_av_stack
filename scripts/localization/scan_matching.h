#include "helper.h"
#include "rclcpp/rclcpp.hpp"
#include <Eigen/Geometry>
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <pcl/visualization/pcl_visualizer.h>
#include <string>

#ifndef SCANMATCHING_H
#define SCANMATCHING_H

class Scan_Matching : public rclcpp::Node
{
protected:
	PointCloudT::Ptr target;
	std::string topic;
	bool viz = false;
	pcl::visualization::PCLVisualizer::Ptr viewer;

public:
	Scan_Matching(PointCloudT::Ptr target, std::string topic);
	virtual void set_map(PointCloudT::Ptr target);
	virtual void get_transform(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg);
	virtual void enable_viz();
	rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub;
};

#endif