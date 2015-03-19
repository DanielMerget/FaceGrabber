#pragma once
#include "stdafx.h"
#undef max
#undef min
#include <pcl/visualization/cloud_viewer.h>
#include <condition_variable>
#include <thread>
#include <mutex>
#include <boost/signals.hpp>

class PCLViewer
{
public:
	PCLViewer(int cloudCount, std::string viewerName = "PCLViewer");
	~PCLViewer();
	
	void updateColoredClouds(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clouds);

	void updateNonColoredClouds(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clouds);

	void stopViewer();

	void useColoredCloud(bool useColored);

	void setNumOfClouds(int numOfClouds);
private:
	void pushNewColoredCloudAtIndex(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, int index);

	void pushNewNonColoredCloudAtIndex(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int index);

	void updateLoop();
	void updateColoredCloud(int cloudIndex, std::string cloudID, pcl::visualization::PCLVisualizer::Ptr viewer);
	void updateNonColoredCloud(int cloudIndex, std::string cloudID, pcl::visualization::PCLVisualizer::Ptr viewer);
	
	boost::signals2::signal<void(int cloudIndex, std::string, pcl::visualization::PCLVisualizer::Ptr)> updateCurrentCloudWithIndexAndIdentifier;

	void createViewPortsForViewer(pcl::visualization::PCLVisualizer::Ptr viewer);
	void matchPointCloudsToViewPorts(pcl::visualization::PCLVisualizer::Ptr viewer);

	bool m_isRunning;
	std::string m_viewerName;
	std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>		m_coloredClouds;
	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>		m_nonColoredClouds;

	bool m_viewPortConfigurationChanged;
	std::mutex m_viewPortConfigurationChangedMutex;
	std::vector<bool> m_cloudUpdated;
	std::vector<int> m_viewPorts;
	std::vector<std::string> m_cloudIDs;

	std::thread m_updateThread;
	std::mutex m_cloudMutex;
	int m_cloudCount;

	std::mutex m_useColoredCloudMutex;
	bool m_useColoredCloud;
	bool m_cloudsUpdated;
	std::condition_variable m_cloudUpdate;
};

