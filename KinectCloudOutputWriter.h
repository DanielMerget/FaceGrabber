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
#include <boost/signals2.hpp>

template < class PointCloudType >
class KinectCloudOutputWriter
{
public:
	
	KinectCloudOutputWriter();
	
	~KinectCloudOutputWriter();
	
	void updateCloudThreated(boost::shared_ptr<const pcl::PointCloud<PointCloudType>> cloud);
		
	void startWritingClouds(int threadsToStart);
	void stopWritingClouds();

	void setRecordingConfiguration(RecordingConfigurationPtr recordingConfiguration);
private:
	
	boost::signals2::signal<void(std::string, const pcl::PointCloud<PointCloudType>&)> writeCloudToDisk;
	//boost::signal2::signal<void<std::string, const pcl::PointCloud<PointCloudType>& >> writeCloudToDisk;
	
	void writeCloudToFile(int index);

	void pushCloud(boost::shared_ptr<const pcl::PointCloud<PointCloudType>> cloud);

	
	struct PointCloudMeasurement{
		boost::shared_ptr<const pcl::PointCloud<PointCloudType>> cloud;
		int index;
	};

	bool m_notified;
	std::vector<std::thread> m_writerThreads;
	std::queue<PointCloudMeasurement> m_clouds;
	std::condition_variable m_checkCloud;
	std::mutex m_lockCloud;

	RecordingConfigurationPtr m_recordingConfiguration;
	

	bool m_running;
	int m_cloudCount;

	
};

