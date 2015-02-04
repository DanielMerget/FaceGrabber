#pragma once
#undef max
#undef min
#include <pcl/visualization/cloud_viewer.h>
#include <thread>
#include <mutex>
class PCLViewer
{
public:
	PCLViewer();
	~PCLViewer();
	
	void updateCloudThreated(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud);

	void updateCloud(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud);
	bool isStopped();

private:

	pcl::visualization::PCLVisualizer viewer;
	std::vector<std::thread> m_updateThreads;
	std::mutex m_viewerMutex;
};

