#include "../include/icp.h"

int main(int argc, char ** argv) 
{
	const std::string pcd_source_path = "../../resources/pcd/color_pc_data1.pcd";
	const std::string pcd_target_path = "../../resources/pcd/color_pc_data2.pcd";
	// four option: icpPlaneToPlane, icpPointToPlane, PointToPlane, PointToPoint
	const std::string option = "icpPlaneToPlane"; 
	return ICP_set::demo(pcd_source_path, pcd_target_path, option);
}