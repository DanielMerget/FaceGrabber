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
	void startCloudUpdateThread();
	void startReaderThreads();
	void stopReaderThreads();

	void join();

	boost::signal<void(void)> playbackFinished;

	boost::signal<void(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)> cloudUpdated;

	void setPlaybackConfiguration(PlaybackConfigurationPtr playbackConfig);
private:

	bool isBufferAtIndexSet(const int index);
	
	void readPLYFile(const int index);
	void updateThreadFunc();
	void printMessage(std::string msg);

	boost::signal<void(std::string, pcl::PointCloud<pcl::PointXYZRGB>&)> readCloudFromDisk;
	std::mutex m_printMutex;
	std::mutex m_cloudBufferMutex;
	std::condition_variable m_cloudBufferFree;
	std::condition_variable m_cloudBufferUpdated;
	std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> m_cloudBuffer;
	std::vector<std::thread> m_readerThreads;
	
	std::thread m_updateThread;
	bool m_isPlaybackRunning;
	const int m_bufferSize;

	PlaybackConfigurationPtr m_playbackConfiguration;
};

