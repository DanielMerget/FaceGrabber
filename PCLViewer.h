#pragma once
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
	
	void updateColoredClouds(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr> clouds);
	void updateNonColoredClouds(std::vector<pcl::PointCloud<pcl::PointXYZ>::ConstPtr> clouds);


	void updateColoredCloudThreated(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud, int index);
	void updateNonColoredCloudThreated(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, int index);
	void stopViewer();
	void useColoredCloud(bool useColored);
	void setNumOfClouds(int numOfClouds);
private:
	void pushNewColoredCloudAtIndex(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud, int index);
	void pushNewNonColoredCloudAtIndex(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, int index);

	void updateLoop();
	void updateColoredCloud(int cloudIndex, std::string cloudID, pcl::visualization::PCLVisualizer::Ptr viewer);
	void updateNonColoredCloud(int cloudIndex, std::string cloudID, pcl::visualization::PCLVisualizer::Ptr viewer);
	
	boost::signals2::signal<void(int cloudIndex, std::string, pcl::visualization::PCLVisualizer::Ptr)> updateCurrentCloudWithIndexAndIdentifier;

	void createViewPortsForViewer(pcl::visualization::PCLVisualizer::Ptr viewer);
	void matchPointCloudsToViewPorts(pcl::visualization::PCLVisualizer::Ptr viewer);

	bool m_isRunning;
	std::string m_viewerName;
	std::vector<pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr>	m_coloredClouds;
	std::vector<pcl::PointCloud<pcl::PointXYZ>::ConstPtr>		m_nonColoredClouds;

	//pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr m_currentCloud1;
	//pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr m_currentCloud2;
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

