#pragma once
#include <queue>
#include <condition_variable>
#include <mutex>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <thread>
#include <vector>

class KinectCloudOutputWriter
{
public:
	KinectCloudOutputWriter();
	~KinectCloudOutputWriter();
	
	void updateCloudThreated(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud);
	void startWritingClouds();
private:
	
	void writeCloudToFile(int index);
	void pushCloud(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud);
	
	struct PointCloudMeasurement{
		pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud;
		int index;
	};

	bool m_notified;
	std::vector<std::thread> m_writerThreads;
	std::vector<std::thread> m_updateThreads;
	std::queue<PointCloudMeasurement> m_clouds;
	std::condition_variable m_checkCloud;
	std::mutex m_lockCloud;
	bool m_running;
	int m_cloudCount;

	
};

