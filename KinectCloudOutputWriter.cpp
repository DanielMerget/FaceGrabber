#include "KinectCloudOutputWriter.h"
#include <pcl/io/ply_io.h>

#include <pcl/io/pcd_io.h>

//template < class PointCloudType >
//KinectCloudOutputWriter< PointCloudType >::KinectCloudOutputWriter() :
//	m_running(false),
//	m_notified(false),
//	m_cloudCount(0),
//	m_clouds(),
//	m_writerThreads()
//{
//}
//
//template < typename PointCloudType >
//KinectCloudOutputWriter< PointCloudType >::~KinectCloudOutputWriter()
//{
//
//	m_running = false;
//	for (auto& thread : m_writerThreads){
//		thread.join();
//	}
//}

//template < typename PointCloudType >
//void KinectCloudOutputWriter< PointCloudType >::updateCloudThreated(boost::shared_ptr<const pcl::PointCloud<PointCloudType>> cloud)
//{
//	if (!m_running){
//		return;
//	}
//	std::async(std::launch::async, &KinectCloudOutputWriter::pushCloud, this, cloud);
//}


//template < typename PointCloudType >
//void KinectCloudOutputWriter<PointCloudType>::pushCloud(boost::shared_ptr<const pcl::PointCloud<PointCloudType>> cloudToPush)
//{
//	std::unique_lock<std::mutex> cloudLocker(m_lockCloud);
//	if (m_cloudCount >= numOfFilesToWrite || !m_running){
//		m_running = false;
//		m_checkCloud.notify_all();
//		return;
//	}
//
//	PointCloudMeasurement cloudMeasurement;
//	cloudMeasurement.cloud = cloudToPush;
//	cloudMeasurement.index = m_cloudCount;
//	m_clouds.push(cloudMeasurement);
//	m_cloudCount++;
//	m_checkCloud.notify_all();
//}

template < typename PointCloudType >
void KinectCloudOutputWriter< PointCloudType >::startWritingClouds()
{
	const int threadsToStartCount = numOfFilesToWrite;
	m_running = true;
	//5 threads => 17,71 secs
	for (int i = 0; i < 5; i++){
		m_writerThreads.push_back(std::thread(&KinectCloudOutputWriter::writeCloudToFile, this, i));
	}
}
template < typename PointCloudType >
void KinectCloudOutputWriter<PointCloudType>::stopWritingClouds()
{
	std::unique_lock<std::mutex> cloudLocker(m_lockCloud);
	m_running = false;
	m_checkCloud.notify_all();
}

template < typename PointCloudType >
void KinectCloudOutputWriter<PointCloudType>::writeCloudToFile(int index)
{
	bool cloudIsEmpty = false;
	while (m_running || !cloudIsEmpty)
	{

		std::unique_lock<std::mutex> cloudLocker(m_lockCloud);
		while (m_clouds.empty() && m_running){
			if (!m_checkCloud.wait_for(cloudLocker, std::chrono::milliseconds(100)) && !m_running){
				return;
			}
		}
		
		if (m_clouds.empty()){
			continue;
		}
		auto cloudMeasurement = m_clouds.front();
		std::stringstream fileName;
		//fileName << "Cloud_" << cloudMeasurement.index  << ".ply";
		fileName << "Cloud_" << cloudMeasurement.index << ".pcd";
		m_clouds.pop();
		cloudIsEmpty = m_clouds.empty();
		cloudLocker.unlock();

		//pcl::io::savePLYFile(fileName.str(), *cloudMeasurement.cloud, false);
		//pcl::io::savePCDFileASCII(fileName.str(), *cloudMeasurement.cloud);
		
		pcl::io::savePCDFileBinaryCompressed(fileName.str(), *cloudMeasurement.cloud);
	}
}