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

	boost::signals2::signal<void(std::wstring)> updateStatus;
private:
	void createAndStartThreadForIndex(int index, int numOfThreads);
	void startReaderThreads();

	bool isBufferAtIndexSet(const int index);
	
	
	void printMessage(std::string msg);
	

	void readerFinishedReadingAFile();

	std::mutex m_printMutex;

	
	std::vector<std::thread> m_readerThreads;


	bool m_isPlaybackRunning;
	
	std::mutex m_readMutex;

	std::shared_ptr<Buffer<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>> m_buffer;

	int m_numOfFilesRead;
	std::mutex m_numOfFilesReadMutex;

	std::thread m_updateThread;
	std::vector<std::shared_ptr<PCLInputReaderWorkerThread>> m_inputReaderWorkerThreads;
	PlaybackConfigurationPtr m_playbackConfiguration;
};


