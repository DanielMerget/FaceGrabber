#pragma once
#include "stdafx.h"

#include <memory>
#include "Buffer.h"
#include "PlaybackConfiguration.h"
#include <boost/signals2.hpp>
template < typename PointType >
class PCLInputReaderWorkerThread 
{
public:
	PCLInputReaderWorkerThread();
	~PCLInputReaderWorkerThread();

	void readCloudData(const int index, const int step, std::vector<CloudFile> cloudFilesToPlay, RecordingFileFormat format);
	void stopReading();
	void setBuffer(std::shared_ptr<Buffer<boost::shared_ptr<pcl::PointCloud<PointType>>>> buffer);

	boost::signals2::signal<void(void)> finishedReadingAFile;
private:
	void printMessage(std::string msg);
	bool m_isPlaybackRunning;
	std::shared_ptr<Buffer<boost::shared_ptr<pcl::PointCloud<PointType>>>> m_buffer;
};

