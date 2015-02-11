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
	PCLViewer(std::string viewerName = "PCLViewer");
	~PCLViewer();
	
	void updateCloudThreated(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud, int index);
	void stopViewer();
private:
	void updateCloud(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud, int index);
	void updateLoop();
	
	bool m_isRunning;
	std::string m_viewerName;
	pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr m_currentCloud1;
	pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr m_currentCloud2;

	std::thread m_updateThread;
	std::mutex m_cloudMutex;

	std::condition_variable m_cloudUpdate;
};

