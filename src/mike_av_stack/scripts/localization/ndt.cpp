#include "ndt.h"

NDT::NDT(PointCloudT::Ptr target, Pose startingPose, int iterations){
    this.target = target;
    this.startingPost = startingPose;
    this.iterations = iterations;

    ndt.setTransformationEpsilon(0.0001);
    ndt.setInputTarget(mapCloud);
    ndt.setResolution(1);
    ndt.setStepSize(1);
}

void ICP::set_map(PointCloudT::Ptr t){
    target = t;
} 

Eigen::Matrix4d NDT::get_transform(const sensor_msgs::PointCloud2ConstPtr& source){

  	Eigen::Matrix4f init_guess = transform3D(startingPose.rotation.yaw, startingPose.rotation.pitch, startingPose.rotation.roll, startingPose.position.x, startingPose.position.y, startingPose.position.z).cast<float>();
  
  	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ndt(new pcl::PointCloud<pcl::PointXYZ>);
  	
  	ndt.setMaximumIterations(iterations);
  	ndt.setInputSource(source);
  	ndt.align(*cloud_ndt, init_guess);
  	Eigen::Matrix4d transformation_matrix = ndt.getFinalTransformation().cast<double>();
  
  	return transformation_matrix;
}