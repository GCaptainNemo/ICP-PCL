#include <iostream>
#include <string>
#define BOOST_TYPEOF_EMULATION
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/time.h>   // TicToc
#include "../include/plane_fitting.h"

typedef pcl::PointXYZ PointT;
typedef pcl::PointXYZRGB PointColor;

typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointCloud<PointColor> PointCloudColor;


bool next_iteration = false;

void
print4x4Matrix(const Eigen::Matrix4d & matrix)
{
	printf("Rotation matrix :\n");
	printf("    | %6.3f %6.3f %6.3f | \n", matrix(0, 0), matrix(0, 1), matrix(0, 2));
	printf("R = | %6.3f %6.3f %6.3f | \n", matrix(1, 0), matrix(1, 1), matrix(1, 2));
	printf("    | %6.3f %6.3f %6.3f | \n", matrix(2, 0), matrix(2, 1), matrix(2, 2));
	printf("Translation vector :\n");
	printf("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix(0, 3), matrix(1, 3), matrix(2, 3));
}

void
keyboardEventOccurred(const pcl::visualization::KeyboardEvent& event,
	void* nothing)
{
	if (event.getKeySym() == "space" && event.keyDown())
		next_iteration = true;
	else if(event.getKeySym() == "q") 
	{
		next_iteration = false;
	}
}

void xyzrgb2xyz(PointCloudColor::Ptr color_pc, PointCloudT::Ptr pc)
{
	int M = color_pc->points.size();
	cout << "input size is:" << M << endl;

	for (int i = 0; i < M; i++)
	{
		PointT p;
		p.x = color_pc->points[i].x;
		p.y = color_pc->points[i].y;
		p.z = color_pc->points[i].z;
		pc->points.push_back(p);
	}
	pc->width = 1;
	pc->height = M;
}

template <class pointcloudtype>
void read_pcds(pointcloudtype cloud_target, const char * target_address)
{
	if (pcl::io::loadPCDFile(target_address, *cloud_target) == -1)
	{
		PCL_ERROR("Error loading cloud %s.\n");
	}
}

void transform(double theta, PointCloudT::Ptr cloud_target, PointCloudT::Ptr cloud_source) 
{
	Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity();
	// A rotation matrix 
	transformation_matrix(0, 0) = cos(theta);
	transformation_matrix(0, 1) = -sin(theta);
	transformation_matrix(1, 0) = sin(theta);
	transformation_matrix(1, 1) = cos(theta);

	// A translation on Z axis (0.4 meters)
	transformation_matrix(2, 3) = 0.4;
	pcl::transformPointCloud(*cloud_target, *cloud_source, transformation_matrix);

	// Display in terminal the transformation matrix
	std::cout << "Applying this rigid transformation to: cloud_in -> cloud_icp" << std::endl;
	print4x4Matrix(transformation_matrix);
}

template <class POINTCLOUDPTR>
void transform_source(POINTCLOUDPTR cloud_source)
{
	Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity();
	// A rotation matrix 
	transformation_matrix(0, 0) = 0.99440256;
	transformation_matrix(1, 0) = -0.10628782;
	transformation_matrix(2, 0) = -0.02411303;

	transformation_matrix(0, 1) = 0.10218425;
	transformation_matrix(1, 1) = 0.98646072;
	transformation_matrix(2, 1) = -0.13140601;
	
	transformation_matrix(0, 2) = 0.03774522;
	transformation_matrix(1, 2) = 0.1280165;
	transformation_matrix(2, 2) = 0.99155707;

	/*transformation_matrix(1, 3) = -0.15136226;
	transformation_matrix(2, 3) = -0.1868385;
	transformation_matrix(3, 3) = -0.02836975;
*/
	transformation_matrix(1, 3) = -6.2303412e-04;
	transformation_matrix(2, 3) = 5.3596579e-04;
	transformation_matrix(3, 3) = 2.5333374e-06;
	// transformation_matrix = transformation_matrix.inverse();

	pcl::transformPointCloud(*cloud_source, *cloud_source, transformation_matrix);

	// Display in terminal the transformation matrix
	std::cout << "Applying this rigid transformation to: cloud_in -> cloud_icp" << std::endl;
	print4x4Matrix(transformation_matrix);
}

int visualize(PointCloudColor::Ptr cloud_source_color, PointCloudColor::Ptr cloud_target_color, 
	PointCloudColor::Ptr cloud_tr_color)
{
	PointCloudT::Ptr cloud_source(new PointCloudT);
	PointCloudT::Ptr cloud_target(new PointCloudT);
	PointCloudT::Ptr cloud_tr(new PointCloudT);
	xyzrgb2xyz(cloud_source_color, cloud_source);
	xyzrgb2xyz(cloud_target_color, cloud_target);
	xyzrgb2xyz(cloud_tr_color, cloud_tr);



	pcl::console::TicToc time;
	Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity();
	int iterations = 1;
	pcl::IterativeClosestPoint<PointT, PointT> icp;
	icp.setMaximumIterations(iterations);
	icp.setInputSource(cloud_source);
	icp.setInputTarget(cloud_target);
	icp.align(*cloud_source);
	icp.setMaximumIterations(1);  // We set this variable to 1 for the next time we will call .align () function
	std::cout << "Applied " << iterations << " ICP iteration(s) in " << time.toc() << " ms" << std::endl;

	if (icp.hasConverged())
	{
		std::cout << "\nICP has converged, score is " << icp.getFitnessScore() << std::endl;
		std::cout << "\nICP transformation " << iterations << " : cloud_icp -> cloud_in" << std::endl;
		transformation_matrix = icp.getFinalTransformation().cast<double>();
		print4x4Matrix(transformation_matrix);
	}
	else
	{
		PCL_ERROR("\nICP has not converged.\n");
		return (-1);
	}

	// Visualization
	pcl::visualization::PCLVisualizer viewer("ICP demo");
	// Create two vertically separated viewports
	int v1(0);
	int v2(1);
	viewer.createViewPort(0.0, 0.0, 0.5, 1.0, v1);
	viewer.createViewPort(0.5, 0.0, 1.0, 1.0, v2);

	// The color we will be using
	float bckgr_gray_level = 0.0;  // Black
	float txt_gray_lvl = 1.0 - bckgr_gray_level;

	// Original point cloud is white
	pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_in_color_h(cloud_target, (int)255 * txt_gray_lvl, (int)255 * txt_gray_lvl,
		(int)255 * txt_gray_lvl);
	viewer.addPointCloud(cloud_target, cloud_in_color_h, "cloud_in_v1", v1);
	viewer.addPointCloud(cloud_target, cloud_in_color_h, "cloud_in_v2", v2);

	// Transformed point cloud is green
	pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_tr_color_h(cloud_tr, 20, 180, 20);
	viewer.addPointCloud(cloud_tr, cloud_tr_color_h, "cloud_tr_v1", v1);

	// ICP aligned point cloud is red
	pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_icp_color_h(cloud_source, 180, 20, 20);
	
	viewer.addPointCloud(cloud_source, cloud_icp_color_h, "cloud_icp_v2", v2);

	// Adding text descriptions in each viewport
	viewer.addText("White: Target point cloud\nGreen: Source point cloud", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "icp_info_1", v1);
	//viewer.addText("White: Target point cloud\nRed: ICP aligned point cloud", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "icp_info_2", v2);
	viewer.addText("White: Target point cloud\nRed: Transform Source point cloud used IMU", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "icp_info_2", v2);

	std::stringstream ss;
	ss << iterations;
	std::string iterations_cnt = "ICP iterations = " + ss.str();
	viewer.addText(iterations_cnt, 10, 60, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "iterations_cnt", v2);

	// Set background color
	viewer.setBackgroundColor(bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v1);
	viewer.setBackgroundColor(bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v2);

	// Set camera position and orientation
	viewer.setCameraPosition(-3.68332, 2.94092, 5.71266, 0.289847, 0.921947, -0.256907, 0);
	viewer.setSize(1280, 1024);  // Visualiser window size

	// Register keyboard callback :
	viewer.registerKeyboardCallback(&keyboardEventOccurred, (void*)NULL);

	// Display the visualiser
	while (!viewer.wasStopped())
	{
		viewer.spinOnce();

		// The user pressed "space" :
		if (next_iteration)
		{
			// The Iterative Closest Point algorithm
			time.tic();
			icp.align(*cloud_source);
			std::cout << "Applied 1 ICP iteration in " << time.toc() << " ms" << std::endl;

			if (icp.hasConverged())
			{
				printf("\033[11A");  // Go up 11 lines in terminal output.
				printf("\nICP has converged, score is %+.0e\n", icp.getFitnessScore());
				std::cout << "\nICP transformation " << ++iterations << " : cloud_icp -> cloud_in" << std::endl;
				transformation_matrix *= icp.getFinalTransformation().cast<double>();  // WARNING /!\ This is not accurate! For "educational" purpose only!
				print4x4Matrix(transformation_matrix);  // Print the transformation between original pose and current pose

				ss.str("");
				ss << iterations;
				std::string iterations_cnt = "ICP iterations = " + ss.str();
				viewer.updateText(iterations_cnt, 10, 60, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "iterations_cnt");
				viewer.updatePointCloud(cloud_source, cloud_icp_color_h, "cloud_icp_v2");
			}
			else
			{
				PCL_ERROR("\nICP has not converged.\n");
				return (-1);
			}
		}
		next_iteration = false;
	}
	std::cout << "after icp iteration " << cloud_source->points.size() << std::endl;
}

int visualize(PointCloudT::Ptr cloud_source, PointCloudT::Ptr cloud_target, PointCloudT::Ptr cloud_tr)
{
	
	pcl::console::TicToc time;
	Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity();
	int iterations = 1;
	pcl::IterativeClosestPoint<PointT, PointT> icp;
	icp.setMaximumIterations(iterations);
	icp.setInputSource(cloud_source);
	icp.setInputTarget(cloud_target);
	icp.align(*cloud_source);
	icp.setMaximumIterations(1);  // We set this variable to 1 for the next time we will call .align () function
	std::cout << "Applied " << iterations << " ICP iteration(s) in " << time.toc() << " ms" << std::endl;

	if (icp.hasConverged())
	{
		std::cout << "\nICP has converged, score is " << icp.getFitnessScore() << std::endl;
		std::cout << "\nICP transformation " << iterations << " : cloud_icp -> cloud_in" << std::endl;
		transformation_matrix = icp.getFinalTransformation().cast<double>();
		print4x4Matrix(transformation_matrix);
	}
	else
	{
		PCL_ERROR("\nICP has not converged.\n");
		return (-1);
	}

	// Visualization
	pcl::visualization::PCLVisualizer viewer("ICP demo");
	// Create two vertically separated viewports
	int v1(0);
	int v2(1);
	viewer.createViewPort(0.0, 0.0, 0.5, 1.0, v1);
	viewer.createViewPort(0.5, 0.0, 1.0, 1.0, v2);

	// The color we will be using
	float bckgr_gray_level = 0.0;  // Black
	float txt_gray_lvl = 1.0 - bckgr_gray_level;

	// Original point cloud is white
	pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_in_color_h(cloud_target, (int)255 * txt_gray_lvl, (int)255 * txt_gray_lvl,
		(int)255 * txt_gray_lvl);
	viewer.addPointCloud(cloud_target, cloud_in_color_h, "cloud_in_v1", v1);
	viewer.addPointCloud(cloud_target, cloud_in_color_h, "cloud_in_v2", v2);

	// Transformed point cloud is green
	pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_tr_color_h(cloud_tr, 20, 180, 20);
	viewer.addPointCloud(cloud_tr, cloud_tr_color_h, "cloud_tr_v1", v1);

	// ICP aligned point cloud is red
	pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_icp_color_h(cloud_source, 180, 20, 20);

	viewer.addPointCloud(cloud_source, cloud_icp_color_h, "cloud_icp_v2", v2);

	// Adding text descriptions in each viewport
	viewer.addText("White: Target point cloud\nGreen: Source point cloud", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "icp_info_1", v1);
	//viewer.addText("White: Target point cloud\nRed: ICP aligned point cloud", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "icp_info_2", v2);
	viewer.addText("White: Target point cloud\nRed: Transform Source point cloud used IMU", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "icp_info_2", v2);

	std::stringstream ss;
	ss << iterations;
	std::string iterations_cnt = "ICP iterations = " + ss.str();
	viewer.addText(iterations_cnt, 10, 60, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "iterations_cnt", v2);

	// Set background color
	viewer.setBackgroundColor(bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v1);
	viewer.setBackgroundColor(bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v2);

	// Set camera position and orientation
	viewer.setCameraPosition(-3.68332, 2.94092, 5.71266, 0.289847, 0.921947, -0.256907, 0);
	viewer.setSize(1280, 1024);  // Visualiser window size

	// Register keyboard callback :
	viewer.registerKeyboardCallback(&keyboardEventOccurred, (void*)NULL);

	// Display the visualiser
	while (!viewer.wasStopped())
	{
		viewer.spinOnce();

		// The user pressed "space" :
		if (next_iteration)
		{
			// The Iterative Closest Point algorithm
			time.tic();
			icp.align(*cloud_source);
			std::cout << "Applied 1 ICP iteration in " << time.toc() << " ms" << std::endl;

			if (icp.hasConverged())
			{
				printf("\033[11A");  // Go up 11 lines in terminal output.
				printf("\nICP has converged, score is %+.0e\n", icp.getFitnessScore());
				std::cout << "\nICP transformation " << ++iterations << " : cloud_icp -> cloud_in" << std::endl;
				transformation_matrix *= icp.getFinalTransformation().cast<double>();  // WARNING /!\ This is not accurate! For "educational" purpose only!
				print4x4Matrix(transformation_matrix);  // Print the transformation between original pose and current pose

				ss.str("");
				ss << iterations;
				std::string iterations_cnt = "ICP iterations = " + ss.str();
				viewer.updateText(iterations_cnt, 10, 60, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "iterations_cnt");
				viewer.updatePointCloud(cloud_source, cloud_icp_color_h, "cloud_icp_v2");
			}
			else
			{
				PCL_ERROR("\nICP has not converged.\n");
				return (-1);
			}
		}
		next_iteration = false;
	}
	std::cout << "after icp iteration " << cloud_source->points.size() << std::endl;
}


int main()
{
	// The point clouds we will be using

	//PointCloudT::Ptr cloud_target(new PointCloudT);  // Original point cloud
	//PointCloudT::Ptr cloud_tr(new PointCloudT);  // Transformed point cloud
	//PointCloudT::Ptr cloud_source(new PointCloudT);  // ICP output point cloud
	PointCloudColor::Ptr cloud_target(new PointCloudColor);  // Original point cloud
	PointCloudColor::Ptr cloud_tr(new PointCloudColor);  // Transformed point cloud
	PointCloudColor::Ptr cloud_source(new PointCloudColor);  // ICP output point cloud


	//const char * target_address = "../resources/pcd/70.pcd";
	//plane_fitting(target_address, 900);
	//const char * source_address = "../resources/pcd/70.pcd";
	//read_pcds(cloud_source, source_address);
	
	const char * target_address = "../../resources/pcd/color_pc_data1.pcd";

	//plane_fitting(target_address, 900);
	read_pcds(cloud_target, target_address);

	const char * source_address = "../../resources/pcd/color_pc_data2.pcd";

	read_pcds(cloud_source, source_address);
	
	//read_pcds(cloud_target, target_address);
	//// transform(M_PI / 8, cloud_target, cloud_source);
	//std::cout << "\nLoaded file " << "mesh_without_color-2.ply" << " (" << cloud_target->size() << " points) " << std::endl;

	*cloud_tr = *cloud_source;  // We backup cloud_source into cloud_tr for later use
	 transform_source(cloud_source);
	if (visualize(cloud_source, cloud_target, cloud_tr) == -1) 
	{
		return -1;
	}
	return (0);
}
