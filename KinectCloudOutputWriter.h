#pragma once
#include "stdafx.h"
#include <queue>
#include <condition_variable>
#include <mutex>

#include <thread>
#include <vector>
#include <future>
#include "RecordingConfiguration.h"
#include "KinectFileWriterThread.h"
#include "CloudMeasurementSource.h"
#include <boost/signals2.hpp>

template < class PointCloudType >
class KinectCloudOutputWriter : CloudMeasurementSource< PointCloudType >
{
public:
	
	KinectCloudOutputWriter();
	
	~KinectCloudOutputWriter();
	
	void updateCloudThreated(boost::shared_ptr<pcl::PointCloud<PointCloudType>> cloud);

	void updateCloudsThreated(std::vector<boost::shared_ptr<pcl::PointCloud<PointCloudType>>> clouds);
		
	void startWritingClouds();
	void stopWritingClouds();

	void setRecordingConfiguration(IRecordingConfigurationPtr recordingConfiguration);


	bool pullData(PointCloudMeasurement<PointCloudType>& measurement);

	boost::signals2::signal<void(std::wstring)> updateStatus;
	boost::signals2::signal<void(void)> writingWasStopped;
private:
	
	bool isMaximumFramesReached();

	void waitForWriterToFinish();

	void pushCloud(boost::shared_ptr<const pcl::PointCloud<PointCloudType>> cloud);

	bool m_notified;
	std::vector<std::thread> m_writerThreads;
	std::vector<std::shared_ptr<KinectFileWriterThread<PointCloudType>>> m_writers;

	std::queue<PointCloudMeasurement<PointCloudType>> m_clouds;

	std::condition_variable m_checkCloud;
	std::mutex m_lockCloud;

	IRecordingConfigurationPtr m_recordingConfiguration;

	bool m_running;
	int m_cloudCount;

	
};

