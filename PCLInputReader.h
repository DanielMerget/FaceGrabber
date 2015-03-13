#pragma once
#include <string>
#include <vector>
#include <thread>
#include <boost/signals2.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <condition_variable>
#include <mutex>
#include "PlaybackConfiguration.h"
#include "Buffer.h"
#include "PCLInputReaderWorkerThread.h"

template <typename PointType>
class PCLInputReader
{
public:
	PCLInputReader();
	~PCLInputReader();
	void startCloudUpdateThread();
	
	void stopReaderThreads();

	void join();

	std::shared_ptr<Buffer< boost::shared_ptr<pcl::PointCloud<PointType>>> > getBuffer();
	void setBuffer(std::shared_ptr<Buffer<boost::shared_ptr<pcl::PointCloud<PointType>>>> buffer);
	void setPlaybackConfiguration(PlaybackConfigurationPtr playbackConfig);
	
	boost::signals2::signal<void(boost::shared_ptr<pcl::PointCloud<PointType> const> cloud)> cloudUpdated;
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

	std::shared_ptr<Buffer<boost::shared_ptr<pcl::PointCloud<PointType>>>> m_buffer;

	int m_numOfFilesRead;
	std::mutex m_numOfFilesReadMutex;

	std::thread m_updateThread;
	std::vector<std::shared_ptr<PCLInputReaderWorkerThread<PointType>>> m_inputReaderWorkerThreads;
	PlaybackConfigurationPtr m_playbackConfiguration;
};


