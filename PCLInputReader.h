#pragma once
#include <string>
#include <vector>
#include <thread>
#include <boost/signals.hpp>
#include <boost/signals2.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <condition_variable>
#include <mutex>
#include "PlaybackConfiguration.h"
#include "Buffer.h"
#include "PCLInputReaderWorkerThread.h"
class PCLInputReader
{
public:
	PCLInputReader();
	~PCLInputReader();
	void startCloudUpdateThread();
	
	void stopReaderThreads();

	void join();
	std::shared_ptr<Buffer< pcl::PointCloud<pcl::PointXYZRGB>::Ptr> > getBuffer();
	void setBuffer(std::shared_ptr<Buffer<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>> buffer);

	

	boost::signal<void(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)> cloudUpdated;

	void setPlaybackConfiguration(PlaybackConfigurationPtr playbackConfig);
private:
	void createAndStartThreadForIndex(int index, int numOfThreads);
	void startReaderThreads();

	bool isBufferAtIndexSet(const int index);
	
	void readCloudData(const int index);
	void updateThreadFunc();
	void printMessage(std::string msg);
	boost::signals2::signal<void(std::string, pcl::PointCloud<pcl::PointXYZRGB>&)> readCloudFromDisk;

	std::mutex m_printMutex;

	
	std::vector<std::thread> m_readerThreads;


	bool m_isPlaybackRunning;
	
	std::mutex m_readMutex;

	std::shared_ptr<Buffer<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>> m_buffer;

	std::thread m_updateThread;
	std::vector<std::shared_ptr<PCLInputReaderWorkerThread>> m_inputReaderWorkerThreads;
	PlaybackConfigurationPtr m_playbackConfiguration;
};


