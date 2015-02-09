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
	void stopViewer();
private:
	void updateCloud(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud);
	void updateLoop();
	
	bool m_isRunning;
	pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr m_currentCloud;
	std::thread m_updateThread;
	std::mutex m_cloudMutex;
	std::condition_variable m_cloudUpdate;
};

