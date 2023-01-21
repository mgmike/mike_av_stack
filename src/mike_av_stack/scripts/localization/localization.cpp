#include "ros/ros.h"
#include <ros/console.h>
#include <thread>
#include <string>
#include <unordered_map>

#include <sensor_msgs/PointCloud2.h>

#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>
#include <pcl/console/time.h>   // TicToc


#include "helper.h"
#include "scan_matching.h"

PointCloudT pclCloud;

void callback(const sensor_msgs::PointCloud2ConstPtr& cloud){
    pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
    pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
    pcl::PCLPointCloud2 cloudfiltered;

}


int main(int argc, char** argv){
    ros::init(argc, argv, "localization");
    ros::NodeHandle nh("~");
    ROS_INFO("Hello world!");

      	// I added the ability to change values through input params for rapid testing
  	enum Registration{ Off, Ndt, Icp};
  	Registration matching = Off;
	string map_name = "map.pcd";
	int iters = 10;
	int dist = 2;
  	int cp_size = 5000;
  	double leafSize = 0.5;
	bool need_to_write = true;

    std::string param;
    int param_int;
    // If the parameter 'map_name' exists and it is type string, then return true
    if (nh.getParam("matching", param)){
        if (strcmp(param, "ndt") == 0){
            matching = Ndt;
        } else if (strcmp(param, "icp") == 0){
            matching = Icp;
        }
    }
    if (nh.getParam("iters", param_int)){
        iters = param_int;
    }
    if (nh.getParam("map_name", param)){
        map_name = param;
    }

    ros::spin();

    // Initialize visualization
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  	viewer->setBackgroundColor (0, 0, 0);

    // Load map
	PointCloudT::Ptr mapCloud(new PointCloudT);
  	pcl::io::loadPCDFile(map_name, *mapCloud);
  	cout << "Loaded " << mapCloud->points.size() << " data points from " << map_name << endl;
	renderPointCloud(viewer, mapCloud, "map", Color(0,0,1));

    // Set up local vehicle representation
    auto vehicle = boost::static_pointer_cast<cc::Vehicle>(ego_actor);
	Pose pose(Point(0,0,0), Rotate(0,0,0));



    typename pcl::PointCloud<PointT>::Ptr cloudFiltered (new pcl::PointCloud<PointT>);
	typename pcl::PointCloud<PointT>::Ptr scanCloud (new pcl::PointCloud<PointT>);

    ros::Subscriber sub = n.subscribe("/carla/ego_vehicle/lidar/lidar1/point_cloud", 100, callback);

    return 0;
}