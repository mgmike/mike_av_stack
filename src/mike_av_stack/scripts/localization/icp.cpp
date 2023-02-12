#include "icp.h"

ICP::ICP(PointCloudT::Ptr t, Pose sp, int iter): Scan_Matching(t), startingPose(sp), iterations(iter) {
	ROS_INFO("In ICP!");
    initTransform = transform3D(startingPose.rotation.yaw, startingPose.rotation.pitch, startingPose.rotation.roll, startingPose.position.x, startingPose.position.y, startingPose.position.z);
}

void ICP::get_transform(const sensor_msgs::PointCloud2ConstPtr& cloud_msg){
	ROS_INFO("Got point cloud!");

    Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity();

	// Create pcl point cloud
	pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
	pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);

	// Convert to pcl
	pcl_conversions::toPCL(*cloud_msg, *cloud);
	PointCloudT::Ptr source(new PointCloudT);
	pcl::fromPCLPointCloud2(*cloud, *source);

	// Make voxel grid
	PointCloudT::Ptr filteredSource(new PointCloudT);
	pcl::VoxelGrid<PointT> vg;
	vg.setInputCloud(source);
	vg.setLeafSize(leafSize, leafSize, leafSize);
	vg.filter(*filteredSource);
 
	// 1. Transform the source to the startingPose
  	PointCloudT::Ptr transformSource(new PointCloudT);
  	pcl::transformPointCloud(*filteredSource, *transformSource, initTransform);
  
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
  	ROS_INFO("WARNING: ICP did not converge");

	if (viz){
		// Transform scan so it aligns with ego's actual pose and render that scan
		PointCloudT::Ptr transformed_scan (new PointCloudT);
		pcl::transformPointCloud(*filteredSource, *transformed_scan, transformation_matrix);
		viewer->removePointCloud("scan");
		renderPointCloud(viewer, transformed_scan, "scan", Color(1,0,0) );

		Pose estimatedPose = getPose(transformation_matrix);

		viewer->removeAllShapes();
		drawCar(viewer,  Color(0,1,0), estimatedPose, 1, 0.35);

		viewer->spinOnce();
	}


	// Do something with this
	// return transformation_matrix;
}