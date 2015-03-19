#include "stdafx.h"
#include "KinectCloudOutputWriter.h"
#include <pcl/io/ply_io.h>

#include <pcl/io/pcd_io.h>
template KinectCloudOutputWriter < pcl::PointXYZRGB > ;
template KinectCloudOutputWriter < pcl::PointXYZ >;

template < class PointCloudType >
KinectCloudOutputWriter< PointCloudType >::KinectCloudOutputWriter() :
	m_running(false),
	m_cloudCount(0),
	m_clouds(),
	m_writerThreads()
{
}

template < typename PointCloudType >
KinectCloudOutputWriter< PointCloudType >::~KinectCloudOutputWriter()
{
	m_running = false;
	for (auto& thread : m_writerThreads){
		if (thread.joinable()){
			thread.join();
		}
	}
}
template < typename PointCloudType >
void KinectCloudOutputWriter< PointCloudType >::waitForWriterToFinish()
{
	updateStatus(L"writing..");
	for (auto& thread : m_writerThreads){
		if (thread.joinable()){
			thread.join();
		}
	}
	updateStatus(L"");
}

template < typename PointCloudType >
void KinectCloudOutputWriter< PointCloudType >::startWritingClouds()
{
	m_running = true;
	m_cloudCount = 0;
	auto threadsToStartCount = m_recordingConfiguration->getThreadCountToStart();

	for (int i = 0; i < threadsToStartCount; i++){
		std::shared_ptr<KinectFileWriterThread< PointCloudType >> writer(new KinectFileWriterThread< PointCloudType >);
		writer->setKinectCloudOutputWriter(this);
		m_writers.push_back(writer);
		m_writerThreads.push_back(std::thread(&KinectFileWriterThread< PointCloudType >::writeCloudToFile, writer, m_recordingConfiguration));
	}

	std::async(std::launch::async, &KinectCloudOutputWriter::waitForWriterToFinish, this);
}
template < typename PointCloudType >
void KinectCloudOutputWriter<PointCloudType>::stopWritingClouds()
{
	m_running = false;
	m_cloudCount = 0;
	m_checkCloud.notify_all();
	writingWasStopped();
}

template < typename PointCloudType >
bool KinectCloudOutputWriter<PointCloudType>::pullData(PointCloudMeasurement<PointCloudType>& measurement)
{
	std::unique_lock<std::mutex> cloudLocker(m_lockCloud);
	while (m_clouds.empty()){
		if (!m_checkCloud.wait_for(cloudLocker, std::chrono::milliseconds(100))){
			if (m_clouds.empty() && !m_running){
				return true;
			}
		}
	}
	if (m_clouds.empty() && !m_running){
		return true;
	}

	measurement = m_clouds.front();
	m_clouds.pop();

	return false;
}

template < typename PointCloudType >
bool KinectCloudOutputWriter<PointCloudType>::isMaximumFramesReached()
{
	if (m_recordingConfiguration->isRecordingDurationUnLimited()){
		return false;
	}
	return m_cloudCount == m_recordingConfiguration->getMaxNumberOfFrames();
}

template < typename PointCloudType >
void KinectCloudOutputWriter<PointCloudType>::pushCloud(boost::shared_ptr<const pcl::PointCloud<PointCloudType>> cloudToPush)
{
	int size = cloudToPush->size();
	
	std::unique_lock<std::mutex> cloudLocker(m_lockCloud);
	
	if (!m_running){
		m_running = false;
		m_checkCloud.notify_all();
		return;
	}

	PointCloudMeasurement<PointCloudType> cloudMeasurement;
	cloudMeasurement.cloud = cloudToPush;
	cloudMeasurement.index = m_cloudCount;
	m_clouds.push(cloudMeasurement);
	m_cloudCount++;
	

	if (isMaximumFramesReached()){
		stopWritingClouds();
	}
	else{
		m_checkCloud.notify_all();
	}
}
template < typename PointCloudType >
void KinectCloudOutputWriter< PointCloudType >::pushCloudsThreated(std::vector<boost::shared_ptr<pcl::PointCloud<PointCloudType>>> clouds)
{
	for (auto cloud : clouds){
		pushCloudThreated(cloud);
	}
}
template < typename PointCloudType >
void KinectCloudOutputWriter< PointCloudType >::pushCloudThreated(boost::shared_ptr<pcl::PointCloud<PointCloudType>> cloud)
{
	if (!m_running){
		return;
	}
	std::async(std::launch::async, &KinectCloudOutputWriter::pushCloud, this, cloud);
}

template < typename PointCloudType >
void KinectCloudOutputWriter<PointCloudType>::setRecordingConfiguration(IRecordingConfigurationPtr recordingConfiguration)
{
	m_recordingConfiguration = recordingConfiguration;
}