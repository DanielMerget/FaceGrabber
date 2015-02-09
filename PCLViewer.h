#pragma once
#undef max
#undef min
#include <pcl/visualization/cloud_viewer.h>
#include <condition_variable>
#include <thread>
#include <mutex>

class PCLViewer
{
public:
	PCLViewer();
	~PCLViewer();
	
	void updateCloudThreated(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud);
	bool isStopped();

private:
	void updateCloud(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud);
	void updateLoop();
	

	pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr m_currentCloud;
	std::vector<std::thread> m_updateThreads;
	std::mutex m_cloudMutex;
	std::condition_variable m_cloudUpdate;
};

