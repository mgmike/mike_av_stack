#include "rclcpp/rclcpp.hpp"
#include <thread>
#include <string>
#include <unordered_map>

#include "sensor_msgs/msg/point_cloud2.hpp"

#define BOOST_BIND_NO_PLACEHOLDERS

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>
#include <pcl/console/time.h>   // TicToc
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>

#include "helper.h"
#include "scan_matching.h"
#include "ndt.h"
#include "icp.h"
#include "icps.h"

// For test, remove
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;

void callback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg){
    // RCLCPP_INFO("In Callback");
    // Create pcl point cloud
    pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
    pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
    pcl::PCLPointCloud2 cloudfiltered;

}

class Localization : public rclcpp::Node{
public:
    bool viz;
    int iters;
    int dist;
    int cp_size;
    int leafSize;
    std::string map_name;
    std::string topic;
    std::string algorithm;
    Scan_Matching* scan_matching;
    Localization() : Node("localization"){
        // I added the ability to change values through input params for rapid testing
        string map_directory = "/media/mike/Storage/Documents/autonomous_sim/src/mike_av_stack/scripts/localization/maps/";

        // this->declare_parameter("viz", true);
        // this->declare_parameter("iters", 20);
        // this->declare_parameter("dist", 2);
        // this->declare_parameter("cp_size", 5000);
        // this->declare_parameter("leafSize", 0.5);
        // this->declare_parameter("map_name", "map.pcd");
        // this->declare_parameter("topic", "/carla/ego_vehicle/lidar/lidar1/point_cloud_full");
        // this->declare_parameter("scan_matching_algorithm", "icp");

        // iters = this->get_parameter("iters").as_int();
        // dist = this->get_parameter("dist").as_int();
        // cp_size = this->get_parameter("cp_size").as_int();
        // leafSize = this->get_parameter("leafSize").as_int();
        // map_name = this->get_parameter("map_name").as_string();
        // topic = this->get_parameter("topic").as_string();
        // algorithm = this->get_parameter("scan_matching_algorithm").as_string();

        // Testing for now
        
        algorithm = "icp";

        // Initialize visualization
        // pcl::visualization::PCLVisualizer::Ptr viewer;
        // viewer.reset(new pcl::visualization::PCLVisualizer ("3D Viewer"));
        // viewer->setBackgroundColor (0, 0, 0);

        // Load map
        PointCloudT::Ptr mapCloud(new PointCloudT);
        pcl::io::loadPCDFile(map_directory + map_name, *mapCloud);
        // RCLCPP_INFO_STREAM("Loaded " << mapCloud->points.size() << " data points from " << map_name);
        // Flip the points. For some reason, they are flipped
        PointCloudT::Ptr flipped(new PointCloudT);
        for (auto point : mapCloud->points){
            flipped->points.push_back(PointT(point.x, -1.0 * point.y, point.z));
        }
        mapCloud = flipped;
        // renderPointCloud(viewer, mapCloud, "map", Color(0,0,1));

        // Get gps position
        Pose pose(Point(65.516594,7.808423,0.275307), Rotate(0.855823,0.0,0.0));

        // Assign the type of scan matching algorithm to scan_matching.
        if (algorithm == "ndt"){
            scan_matching = new NDT(mapCloud, topic, pose, iters);
        } else if (algorithm == "icp"){
            scan_matching = new ICP(mapCloud, topic, pose, iters);
        } else if (algorithm == "icps"){
            scan_matching = new ICPS(mapCloud, topic, pose, iters, dist);
        }
        //Move inside later
        RCLCPP_INFO(scan_matching->get_logger(), "Setting up subscriber");

        if (viz){
            scan_matching->enable_viz();
        }
    }
};

class TestSub : public rclcpp::Node
{
    public:
        TestSub()
        : Node("test")
        {
            subscription_ = this->create_subscription<std_msgs::msg::String>("topic", 10, std::bind(&TestSub::test_cb, this, _1));
        }

    private:
        void test_cb(const std_msgs::msg::String::SharedPtr msg) const 
        {
            RCLCPP_INFO(this->get_logger(), "Got msg");
        }
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    // RCLCPP_INFO("Hello world!");

    std::string topic;
    Scan_Matching* scan_matching;
    string map_directory = "/media/mike/Storage/Documents/autonomous_sim/src/mike_av_stack/scripts/localization/maps/";
    string map_name = "map.pcd";
    int iters = 20;
    // Get gps position
    Pose pose(Point(65.516594,7.808423,0.275307), Rotate(0.855823,0.0,0.0));


    // Load map
    PointCloudT::Ptr mapCloud(new PointCloudT);
    pcl::io::loadPCDFile(map_directory + map_name, *mapCloud);
    // RCLCPP_INFO_STREAM("Loaded " << mapCloud->points.size() << " data points from " << map_name);
    // Flip the points. For some reason, they are flipped
    PointCloudT::Ptr flipped(new PointCloudT);
    for (auto point : mapCloud->points){
        flipped->points.push_back(PointT(point.x, -1.0 * point.y, point.z));
    }
    mapCloud = flipped;
    // renderPointCloud(viewer, mapCloud, "map", Color(0,0,1));
    
    // auto localization = std::make_shared<ICP>(ICP(mapCloud, topic, pose, iters));
    auto localization = std::make_shared<TestSub>();
    rclcpp::spin(localization);
    rclcpp::shutdown();

    return 0;
}