#include "../include/plane_fitting.h"


#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <vector>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>


int plane_fitting(const char * address, const int & filter_size)
{
	pcl::PCLPointCloud2::Ptr cloud_blob(new pcl::PCLPointCloud2), cloud_filtered_blob(new pcl::PCLPointCloud2);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);


	// Fill in the cloud data
	pcl::PCDReader reader;
	reader.read(address, *cloud_blob);

	std::cerr << "PointCloud before filtering: " << cloud_blob->width * cloud_blob->height << " data points." << std::endl;

	// Create the filtering object: downsample the dataset using a leaf size of 1cm
	pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
	sor.setInputCloud(cloud_blob);
	sor.setLeafSize(0.01f, 0.01f, 0.01f);
	sor.filter(*cloud_filtered_blob);

	// Convert to the templated PointCloud
	pcl::fromPCLPointCloud2(*cloud_filtered_blob, *cloud_filtered);//转化为模板<Template>点云

	std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height << " data points." << std::endl;

	// Write the downsampled version to disk
	pcl::PCDWriter writer;
	writer.write<pcl::PointXYZ>("downsampled.pcd", *cloud_filtered, false);

	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
	// Create the segmentation object
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	// Optional
	seg.setOptimizeCoefficients(true); //设置对估计的模型参数进行优化处理
	// Mandatory
	seg.setModelType(pcl::SACMODEL_PLANE);//
	seg.setMethodType(pcl::SAC_RANSAC); // 设置用哪个随机参数估计方法
	seg.setMaxIterations(1000);
	seg.setDistanceThreshold(0.01); 
	
	// Create the filtering object
	pcl::ExtractIndices<pcl::PointXYZ> extract;

	int i = 0, nr_points = (int)cloud_filtered->points.size();
	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr > Points;
	// While 30% of the original cloud is still there

	pcl::PointCloud<pcl::PointXYZ>::Ptr point_summation(new pcl::PointCloud<pcl::PointXYZ>);

	while (cloud_filtered->points.size() > 0.3 * nr_points)
	{
		// Segment the largest planar component from the remaining cloud
		pcl::PointCloud<pcl::PointXYZ>::Ptr  cloud_p(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZ>::Ptr  cloud_f(new pcl::PointCloud<pcl::PointXYZ>);
		seg.setInputCloud(cloud_filtered);
		seg.segment(*inliers, *coefficients);
		if (inliers->indices.size() == 0)
		{
			std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
			break;
		}

		// Extract the inliers
		extract.setInputCloud(cloud_filtered);
		extract.setIndices(inliers);
		extract.setNegative(false);
		extract.filter(*cloud_p);
		std::cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;
		if (cloud_p->width * cloud_p->height > filter_size) 
		{
			*point_summation = *point_summation + *cloud_p;
			Points.push_back(cloud_p);
		}

		// std::stringstream ss;
		// ss << "plane_" << i << ".pcd";
		// writer.write<pcl::PointXYZ>(ss.str(), *cloud_p, false);

		// Create the filtering object
		extract.setNegative(true);
		extract.filter(*cloud_f);
		cloud_filtered.swap(cloud_f);//更新
		i++;
	}
	std::stringstream ss;
	ss << "all_plane_" << 70 << ".pcd";
	writer.write<pcl::PointXYZ>(ss.str(), *point_summation, false);

	pcl::visualization::PCLVisualizer viewer("demo");
	int v1(0);
	int v2(1);
	viewer.createViewPort(0.0, 0.0, 1.0, 1.0, v1);

	// The color we will be using
	float bckgr_gray_level = 0.0;  // Black
	float txt_gray_lvl = 1.0 - bckgr_gray_level;
	// Original point cloud is white
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_in_color_h(cloud_filtered, (int)255 * txt_gray_lvl, (int)255 * txt_gray_lvl, (int)255 * txt_gray_lvl);
	viewer.addPointCloud(cloud_filtered, cloud_in_color_h, "cloud_in_v1", v1);
	for (int i = 0; i < Points.size(); i++)
	{
		pcl::PointCloud<pcl::PointXYZ> ::Ptr cloud_p;
		cloud_p = Points.at(i);
		//viewer.addPointCloud(cloud_in, cloud_in_color_h, "cloud_in_v2", v2);
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_out(cloud_p, 255 * (i % 2), 255 * (i % 3), 55);
		char ss[10];
		std::string st = "name";
		sprintf_s(ss, "%d", i);
		st += ss;
		viewer.addPointCloud(cloud_p, cloud_out, st, v1);

	}
	viewer.setSize(1280, 1024);  // Visualiser window size
	//viewer.showCloud(cloud_out);
	while (!viewer.wasStopped())
	{
		viewer.spinOnce();
	}

	return (0);
}

