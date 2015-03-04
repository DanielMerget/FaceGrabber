#pragma once
#include <string>
#include <vector>
#include <thread>
#include <boost/signals.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <condition_variable>
#include <mutex>
#include "PlaybackConfiguration.h"

class PCLInputReader
{
public:
	PCLInputReader(const int bufferSize);
	~PCLInputReader();
	void startReaderThreads();
	void startCloudUpdateThread();
	void join();

	boost::signal<void(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)> cloudUpdated;

	void setPlaybackConfiguration(PlaybackConfigurationPtr playbackConfig);
private:
	bool isBufferAtIndexSet(const int index);
	std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> m_cloudBuffer;
	void readPLYFile(const int index);
	void updateThreadFunc();
	void printMessage(std::string msg);


	std::mutex m_printMutex;
	
	std::mutex m_cloudBufferMutex;
	std::condition_variable m_cloudBufferFree;
	std::condition_variable m_cloudBufferUpdated;

	std::vector<std::thread> m_readerThreads;
	std::thread m_updateThread;

	const int m_bufferSize;

	PlaybackConfigurationPtr m_playbackConfiguration;
};

