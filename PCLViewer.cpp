#include "PCLViewer.h"


PCLViewer::PCLViewer() :
	viewer("Face HD PCL Viewer")
{
	
}


PCLViewer::~PCLViewer()
{
	for (auto& thread : m_updateThreads){
		thread.join();
	}
}

bool PCLViewer::isStopped(){
	return viewer.wasStopped();
}

void PCLViewer::updateCloudThreated(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
{
	m_updateThreads.push_back(std::thread(&PCLViewer::updateCloud, this, cloud));
}

void PCLViewer::updateCloud(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
{
	std::unique_lock<std::mutex> viewerLock(m_viewerMutex);
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
