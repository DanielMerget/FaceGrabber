#pragma once
#include <queue>
#include <condition_variable>
#include <mutex>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <thread>
#include <vector>
#include <future>
#include "RecordingConfiguration.h"
#include "SingleProducerBuffer.h"
#include "KinectFileWriterThread.h"
#include "CloudMeasurementSource.h"
#include <boost/signals2.hpp>

template < class PointCloudType >
class KinectCloudOutputWriter : CloudMeasurementSource< PointCloudType >
{
public:
	
	KinectCloudOutputWriter();
	
	~KinectCloudOutputWriter();
	
	void updateCloudThreated(boost::shared_ptr<const pcl::PointCloud<PointCloudType>> cloud);
		
	void startWritingClouds(int threadsToStart);
	void stopWritingClouds();

	void setRecordingConfiguration(RecordingConfigurationPtr recordingConfiguration);


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

	RecordingConfigurationPtr m_recordingConfiguration;

	bool m_running;
	int m_cloudCount;

	
};

