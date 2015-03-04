#include "KinectCloudOutputWriter.h"
#include <pcl/io/ply_io.h>

#include <pcl/io/pcd_io.h>
template KinectCloudOutputWriter < pcl::PointXYZRGB > ;
template KinectCloudOutputWriter < pcl::PointXYZ >;

template < class PointCloudType >
KinectCloudOutputWriter< PointCloudType >::KinectCloudOutputWriter() :
	m_running(false),
	m_notified(false),
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

static int numOfFilesToWrite = 1000;

template < typename PointCloudType >
void KinectCloudOutputWriter< PointCloudType >::startWritingClouds(int threadsToStart)
{
	const int threadsToStartCount = numOfFilesToWrite;
	m_running = true;
	m_cloudCount = 0;
	//5 threads => 17,71 secs
	for (int i = 0; i < threadsToStart; i++){
		m_writerThreads.push_back(std::thread(&KinectCloudOutputWriter::writeCloudToFile, this, i));
	}
}
template < typename PointCloudType >
void KinectCloudOutputWriter<PointCloudType>::stopWritingClouds()
{
	m_running = false;
	m_cloudCount = 0;
	m_checkCloud.notify_all();
	//for (auto& thread : m_writerThreads){
	//	thread.join();
	//}
	//
	//m_writerThreads.clear();
	
}

template < typename PointCloudType >
void KinectCloudOutputWriter<PointCloudType>::writeCloudToFile(int index)
{
	bool cloudIsEmpty = false;
	do{
		std::unique_lock<std::mutex> cloudLocker(m_lockCloud);
		while (m_clouds.empty()){
			if (!m_checkCloud.wait_for(cloudLocker, std::chrono::milliseconds(100))){
				if (m_clouds.empty() && !m_running){
					return;
				}
			}
		}
		if (m_clouds.empty()){
			return;
		}
		auto cloudMeasurement = m_clouds.front();

		auto filePath = m_recordingConfiguration->getFullRecordingPathString();
		auto fileName = m_recordingConfiguration->getFileNameString();
		auto fileFormatExtension = m_recordingConfiguration->getFileFormatFileExtension();
		std::stringstream outputFileWithPath;
		
		outputFileWithPath << filePath << "\\" << fileName << "_Cloud_" << cloudMeasurement.index << fileFormatExtension;
		
		m_clouds.pop();
		cloudIsEmpty = m_clouds.empty();
		cloudLocker.unlock();
		
		OutputDebugString(index + L"starting to write \n");
		writeCloudToDisk(outputFileWithPath.str(), *cloudMeasurement.cloud);
		OutputDebugString(index + L"stop to write \n" );
	} while (!m_clouds.empty() || m_running);
	OutputDebugString(L"writer finished");

}

template < typename PointCloudType >
void KinectCloudOutputWriter<PointCloudType>::pushCloud(boost::shared_ptr<const pcl::PointCloud<PointCloudType>> cloudToPush)
{
	int size = cloudToPush->size();
	
	std::unique_lock<std::mutex> cloudLocker(m_lockCloud);
	
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

template < typename PointCloudType >
void KinectCloudOutputWriter< PointCloudType >::updateCloudThreated(boost::shared_ptr<const pcl::PointCloud<PointCloudType>> cloud)
{
	if (!m_running){
		return;
	}
	std::async(std::launch::async, &KinectCloudOutputWriter::pushCloud, this, cloud);
}

template < typename PointCloudType >
void KinectCloudOutputWriter<PointCloudType>::setRecordingConfiguration(RecordingConfigurationPtr recordingConfiguration)
{
	m_recordingConfiguration = recordingConfiguration;
	auto recordingType = m_recordingConfiguration->getRecordFileFormat();
	switch (recordingType)
	{
	case PLY:
		writeCloudToDisk.connect(boost::bind(&pcl::io::savePCDFile<PointCloudType>, _1, _2, false));
		break;
	case PLY_BINARY:
		writeCloudToDisk.connect(boost::bind(&pcl::io::savePCDFile<PointCloudType>, _1, _2, true));
		break;
	case PCD:
		writeCloudToDisk.connect(boost::bind(&pcl::io::savePLYFile<PointCloudType>, _1, _2, false));
		break;
	case PCD_BINARY:
		writeCloudToDisk.connect(boost::bind(&pcl::io::savePLYFile<PointCloudType>, _1, _2, true));
		break;
	case RECORD_FILE_FORMAT_COUNT:
		break;
	default:
		break;
	}
}