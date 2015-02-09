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
	std::async(&KinectCloudOutputWriter::pushCloud, this, cloud);
}

void KinectCloudOutputWriter::pushCloud(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloudToPush)
{
	std::unique_lock<std::mutex> cloudLocker(m_lockCloud);
	if (m_cloudCount > 100 || !m_running){
		return;
	}
	//PointCloudMeasurement cloudMeasurement{ cloudToPush, m_cloudCount };	
	PointCloudMeasurement cloudMeasurement;
	cloudMeasurement.cloud = cloudToPush;
	cloudMeasurement.index = m_cloudCount;
	m_clouds.push(cloudMeasurement);
	m_cloudCount++;
	if (m_cloudCount == 100){
		m_running = false;
	}
	m_checkCloud.notify_one();
}

void KinectCloudOutputWriter::startWritingClouds()
{
	const int threadsToStartCount = 5;
	m_running = true;
	for (int i = 0; i < threadsToStartCount; i++){
		m_writerThreads.push_back(std::thread(&KinectCloudOutputWriter::writeCloudToFile, this, i));
	}
}

void KinectCloudOutputWriter::writeCloudToFile(int index)
{
	while (m_running)
	{
		std::unique_lock<std::mutex> cloudLocker(m_lockCloud);
		m_checkCloud.wait(cloudLocker);

		if (!m_clouds.empty())
		{
			auto cloudMeasurement = m_clouds.front();
			std::stringstream fileName;
			fileName << "Cloud_" << cloudMeasurement.index << ".ply";
			pcl::io::savePLYFile(fileName.str(), *cloudMeasurement.cloud, false);
			m_clouds.pop();
		}
		m_notified = false;
	}
}