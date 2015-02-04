#include "PCLViewer.h"


PCLViewer::PCLViewer() :
	viewer("Face HD PCL Viewer")
{
	
}


PCLViewer::~PCLViewer()
{
	
}

bool PCLViewer::isStopped(){
	return viewer.wasStopped();
}

void PCLViewer::updateCloud(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud){
	static bool first = true;
	
	if (first){
		viewer.addPointCloud(cloud, "cloud");
		first = false;
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "cloud");
		viewer.initCameraParameters();
	}
	else{
		viewer.updatePointCloud(cloud, "cloud");
		viewer.spinOnce();
	}
}

void PCLViewer::updateCloudMsg(std::string updateMesg)
{
	std::cout << "Cloud update msg: " << updateMesg << std::endl;
}