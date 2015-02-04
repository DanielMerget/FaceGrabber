#pragma once
#undef max
#undef min
#include <pcl/visualization/cloud_viewer.h>
class PCLViewer
{
public:
	PCLViewer::PCLViewer();
	~PCLViewer();
	
	void updateCloud(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud);
	bool isStopped();
	void updateCloudMsg(std::string updateMesg);
private:
	//pcl::visualization::CloudViewe rviewer;
	pcl::visualization::PCLVisualizer viewer;
};

