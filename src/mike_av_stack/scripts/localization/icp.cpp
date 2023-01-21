#include "icp.h"

ICP::ICP(PointCloudT::Ptr target&, Pose startingPose, int iterations){
    this.target = target;
    this.startingPost = startingPose;
    this.iterations = iterations;

    transformation_matrix = Eigen::Matrix4d::Identity();
    initTransform = transform3D(startingPose.rotation.yaw, startingPose.rotation.pitch, startingPose.rotation.roll, startingPose.position.x, startingPose.position.y, startingPose.position.z);

}

// Make sure this is right
void ICP::set_map(PointCloudT::Ptr t){
    target = t;
} 

vector<int> NN(pcl::KdTreeFLANN<PointT> kdtree, PointCloudT::Ptr source, double dist){
	
	vector<int> associations;

	// This function returns a vector of target indicies that correspond to each source index inorder.
	// E.G. source index 0 -> target index 32, source index 1 -> target index 5, source index 2 -> target index 17, ... 

	// Loop through each transformed source point and using the KDtree find the transformed source point's nearest target point. Append the nearest point to associaitons 
	int i = 0;
	for (PointT pt : source->points){

		vector<int> pointIdxRadiusSearch;
		vector<float> pointRadiusSquaredDistance;
		if (kdtree.radiusSearch(pt, dist, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0){
			associations.push_back(pointIdxRadiusSearch[0]);
        } else {
			associations.push_back(-1);
        }
		i++;
    }
  
	return associations;
}

Eigen::Matrix4d ICP::get_transform(const sensor_msgs::PointCloud2ConstPtr& source){
 

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
		return transformation_matrix;
    }
  	std::cout << "WARNING: ICP did not converge" << std::endl;

	return transformation_matrix;
}