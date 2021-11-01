#pragma once
#include <iostream>
#include <map>
#include <string>
#include <pcl/console/time.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/transformation_estimation_point_to_plane.h>
#include <pcl/visualization/pcl_visualizer.h>

namespace ICP_set 
{
	using ICPFunPtr = Eigen::Matrix4f(*)(pcl::PointCloud<pcl::PointXYZI>::Ptr,
		pcl::PointCloud<pcl::PointXYZI>::Ptr, Eigen::Matrix4f&);

	Eigen::Matrix4f
		PointToPlane(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud1,
			pcl::PointCloud<pcl::PointXYZI>::Ptr cloud2,
			Eigen::Matrix4f &guess);

	Eigen::Matrix4f
		PointToPoint(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_source,
			pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_target,
			Eigen::Matrix4f &guess);
	

	Eigen::Matrix4f
		icpPlaneToPlane(pcl::PointCloud<pcl::PointXYZI>::Ptr src,
			pcl::PointCloud<pcl::PointXYZI>::Ptr tar,
			Eigen::Matrix4f &guess);

	void
	addNormal(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
		pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_with_normals);
	

	Eigen::Matrix4f
		icpPointToPlane(pcl::PointCloud<pcl::PointXYZI>::Ptr src,
			pcl::PointCloud<pcl::PointXYZI>::Ptr tar,
			Eigen::Matrix4f &guess);
	Eigen::Matrix4f
		genTransformation(Eigen::Vector3f &r, Eigen::Vector3f &t);
	

	void displayAngel(Eigen::Matrix4f &transformation);
	Eigen::Matrix4f
		run(pcl::PointCloud<pcl::PointXYZI>::Ptr src,
			pcl::PointCloud<pcl::PointXYZI>::Ptr tar,
			ICPFunPtr icp);
	
	int demo(const std::string &pcd_source_path, const std::string &pcd_target_path, const std::string & option);

	extern std::map<std::string, ICPFunPtr> icp_map;
}