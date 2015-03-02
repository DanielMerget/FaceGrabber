#pragma once
#include <queue>
#include <condition_variable>
#include <mutex>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <thread>
#include <vector>
#include <future>
template < class PointCloudType >
class KinectCloudOutputWriter
{
public:
	//template < class PointCloudType >
	KinectCloudOutputWriter() :
		m_running(false),
		m_notified(false),
		m_cloudCount(0),
		m_clouds(),
		m_writerThreads()
	{
	}
	//~KinectCloudOutputWriter();
	//template < typename PointCloudType >
	~KinectCloudOutputWriter()
	{

		m_running = false;
		for (auto& thread : m_writerThreads){
			thread.join();
		}
	}
	
	//void updateCloudThreated(boost::shared_ptr<const pcl::PointCloud<PointCloudType>> cloud);
	//template < typename PointCloudType >
	void KinectCloudOutputWriter< PointCloudType >::updateCloudThreated(boost::shared_ptr<const pcl::PointCloud<PointCloudType>> cloud)
	{
		if (!m_running){
			return;
		}
		std::async(std::launch::async, &KinectCloudOutputWriter::pushCloud, this, cloud);
	}

	void startWritingClouds();
	void stopWritingClouds();




private:
	
	void writeCloudToFile(int index);

	//void pushCloud(boost::shared_ptr<const pcl::PointCloud<PointCloudType>> cloud);

	
	void pushCloud(boost::shared_ptr<const pcl::PointCloud<PointCloudType>> cloudToPush)
	{
		std::unique_lock<std::mutex> cloudLocker(m_lockCloud);
		const int numOfFilesToWrite = 1000;
		if (m_cloudCount >= numOfFilesToWrite || !m_running){
			m_running = false;
			m_checkCloud.notify_all();
			return;
		}

		PointCloudMeasurement cloudMeasurement;
		cloudMeasurement.cloud = cloudToPush;
		cloudMeasurement.index = m_cloudCount;
		m_clouds.push(cloudMeasurement);
		m_cloudCount++;
		m_checkCloud.notify_all();
	}

	struct PointCloudMeasurement{
		boost::shared_ptr<const pcl::PointCloud<PointCloudType>> cloud;
		int index;
	};

	bool m_notified;
	std::vector<std::thread> m_writerThreads;
	std::queue<PointCloudMeasurement> m_clouds;
	std::condition_variable m_checkCloud;
	std::mutex m_lockCloud;

	

	bool m_running;
	int m_cloudCount;

	
};

