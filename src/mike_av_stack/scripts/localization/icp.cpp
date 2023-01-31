#include "icp.h"

ICP::ICP(PointCloudT::Ptr t, Pose sp, int iter): startingPose(sp), iterations(iter) {
	target = t;

    transformation_matrix = Eigen::Matrix4d::Identity();
    initTransform = transform3D(startingPose.rotation.yaw, startingPose.rotation.pitch, startingPose.rotation.roll, startingPose.position.x, startingPose.position.y, startingPose.position.z);

}

void ICP::get_transform(const sensor_msgs::PointCloud2ConstPtr& cloud_msg){

	// Create pcl point cloud
	pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
	pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
	pcl::PCLPointCloud2 cloudfiltered;

	// Convert to pcl
	pcl_conversions::toPCL(*cloud_msg, *cloud);
	PointCloudT::Ptr source(new PointCloudT);
	pcl::fromPCLPointCloud2(*cloud, *source);
 
	// 1. Transform the source to the startingPose
  	PointCloudT::Ptr transformSource(new PointCloudT);
  	pcl::transformPointCloud(*source, *transformSource, initTransform);
  
	//2. Create the PCL icp object
	pcl::console::TicToc time;
  	time.tic ();
  	pcl::IterativeClosestPoint<PointT, PointT> icp;

	//3. Set the icp object's values
  	icp.setMaximumIterations(iterations);
  	icp.setInputSource(transformSource);
  	icp.setInputTarget(target);
  	icp.setMaxCorrespondenceDistance(2);
  
	//4. Call align on the icp object
  	PointCloudT::Ptr tempSource (new PointCloudT);
  	icp.align(*tempSource);
  
  	if(icp.hasConverged()){
		//std::cout << "\nICP has converged, score is " << icp.getFitnessScore() << std::endl;
		transformation_matrix = icp.getFinalTransformation().cast<double>();
		transformation_matrix = transformation_matrix * initTransform;
		// return transformation_matrix;
    }
  	std::cout << "WARNING: ICP did not converge" << std::endl;

	// Do something with this
	// return transformation_matrix;
}