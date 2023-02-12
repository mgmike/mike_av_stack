#include "scan_matching.h"

Scan_Matching::Scan_Matching(PointCloudT::Ptr t): target(t) {}

void Scan_Matching::set_map(PointCloudT::Ptr t){
    target = t;
} 

void Scan_Matching::get_transform(const sensor_msgs::PointCloud2ConstPtr& cloud_msg){
}

void Scan_Matching::enable_viz(){
	viewer.reset(new pcl::visualization::PCLVisualizer("3D Viewer"));
  	viewer->setBackgroundColor(0, 0, 0);
	renderPointCloud(viewer, target, "map", Color(0,0,1));
	viz = true;
	ROS_INFO("Starting vis");
	// while (!viewer->wasStopped ())
	// {
	// 	std::cout << "Spun" << std::endl;
	// 	viewer->spinOnce (100);
	// 	boost::this_thread::sleep (boost::posix_time::microseconds (100000));
	// }
}

double Score(vector<int> pairs, PointCloudT::Ptr target, PointCloudT::Ptr source, Eigen::Matrix4d transform){
	double score = 0;
	int index = 0;
	for(int i : pairs){
		Eigen::MatrixXd p(4, 1);
		p(0,0) = (*source)[index].x;
		p(1,0) = (*source)[index].y;
		p(2,0) = 0.0;
		p(3,0) = 1.0;
		Eigen::MatrixXd p2 = transform * p;
		PointT association = (*target)[i];
		score += sqrt( (p2(0,0) - association.x) * (p2(0,0) - association.x) + (p2(1,0) - association.y) * (p2(1,0) - association.y) );
		index++;
	}
	return score;
}

