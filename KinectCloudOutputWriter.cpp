#include "KinectCloudOutputWriter.h"
#include <pcl/io/ply_io.h>
#include <future>

KinectCloudOutputWriter::KinectCloudOutputWriter() :
	m_running(false),
	m_notified(false),
	m_cloudCount(0),
	m_clouds(),
	m_writerThreads()
{
}


KinectCloudOutputWriter::~KinectCloudOutputWriter()
{

	m_running = false;
	for (auto& thread : m_writerThreads){
		thread.join();
	}
}

void KinectCloudOutputWriter::updateCloudThreated(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
{
	if (!m_running){
		return;
	}
	//m_updateThreads.push_back(std::thread(&KinectCloudOutputWriter::pushCloud, this, cloud));
	std::async(std::launch::async, &KinectCloudOutputWriter::pushCloud, this, cloud);
}

static int numOfFilesToWrite = 100;
void KinectCloudOutputWriter::pushCloud(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloudToPush)
{
	std::unique_lock<std::mutex> cloudLocker(m_lockCloud);
	if (m_cloudCount >= numOfFilesToWrite || !m_running){
		m_running = false;
		m_checkCloud.notify_all();
		return;
	}
	//PointCloudMeasurement cloudMeasurement{ cloudToPush, m_cloudCount };	
	PointCloudMeasurement cloudMeasurement;
	cloudMeasurement.cloud = cloudToPush;
	cloudMeasurement.index = m_cloudCount;
	m_clouds.push(cloudMeasurement);
	m_cloudCount++;
	//if (m_cloudCount == numOfFilesToWrite){
	//	m_running = false;
	//}
	m_checkCloud.notify_all();
}

void KinectCloudOutputWriter::startWritingClouds()
{
	const int threadsToStartCount = numOfFilesToWrite;
	m_running = true;
	//5 threads => 17,71 secs
	for (int i = 0; i < 5; i++){
		m_writerThreads.push_back(std::thread(&KinectCloudOutputWriter::writeCloudToFile, this, i));
	}
}



void KinectCloudOutputWriter::writeCloudToFile(int index)
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
		fileName << "Cloud_" << cloudMeasurement.index  << ".ply";
		m_clouds.pop();
		cloudIsEmpty = m_clouds.empty();
		cloudLocker.unlock();
		pcl::io::savePLYFile(fileName.str(), *cloudMeasurement.cloud, false);
	}
}