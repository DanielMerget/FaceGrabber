#include "stdafx.h"
#include "PCLViewer.h"
#include <future>
#include <Windows.h>

PCLViewer::PCLViewer(int cloudCount, std::string viewerName) :
	m_cloudCount(cloudCount),
	m_viewerName(viewerName),
	m_coloredClouds(cloudCount),
	m_nonColoredClouds(cloudCount),
	m_useColoredCloud(true),
	m_isRunning(false),
	m_updateThread(&PCLViewer::updateLoop, this),
	m_viewPortConfigurationChanged(false),
	m_cloudsUpdated(false)
{
	useColoredCloud(m_useColoredCloud);
}


PCLViewer::~PCLViewer()
{
	m_updateThread.join();	
}

void PCLViewer::stopViewer()
{
	std::unique_lock<std::mutex> lock(m_cloudMutex);
	m_isRunning = false;
	m_cloudUpdate.notify_all();
}

void PCLViewer::updateNonColoredClouds(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clouds)
{
	std::unique_lock<std::mutex> lock(m_cloudMutex);
	auto it = m_nonColoredClouds.begin();
	for (auto cloud : clouds){
		*it = cloud;
		it++;
	}
	m_isRunning = true;
	m_cloudsUpdated = true;
	m_cloudUpdate.notify_all();
}

void PCLViewer::updateColoredClouds(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clouds)
{
	std::unique_lock<std::mutex> lock(m_cloudMutex);
	auto it = m_coloredClouds.begin();
	for (auto cloud : clouds){
		*it = cloud;
		it++;
	}
	m_isRunning = true;
	m_cloudsUpdated = true;
	m_cloudUpdate.notify_all();
}


void PCLViewer::createViewPortsForViewer(pcl::visualization::PCLVisualizer::Ptr viewer)
{
	m_viewPorts.clear();
	m_cloudIDs.clear();
	m_viewPorts.resize(m_cloudCount);
	m_cloudIDs.resize(m_cloudCount);
	float horizontalSplitPlaneWidth = 1.0f / m_cloudCount;
	for (int i = 0; i < m_cloudCount; i++){
		float xMin = horizontalSplitPlaneWidth * i;
		float xMax = horizontalSplitPlaneWidth * (i + 1);
		viewer->createViewPort(xMin, 0, xMax, 1.0, m_viewPorts[i]);
		m_cloudIDs[i] = std::to_string(i);
	}
	for (int i = 0; i < m_cloudCount; i++){
		double greyVal = 0.9;
		viewer->setBackgroundColor(greyVal, greyVal, greyVal, i);
		
	}
	
}

void PCLViewer::matchPointCloudsToViewPorts(pcl::visualization::PCLVisualizer::Ptr viewer)
{

	for (int i = 0; i < m_cloudCount; i++){
		if (m_useColoredCloud){
			viewer->addPointCloud(m_coloredClouds[i], m_cloudIDs[i], m_viewPorts[i]);
		}
		else{
			pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> blackPoints(m_nonColoredClouds[i], 0, 0, 0);
			viewer->addPointCloud(m_nonColoredClouds[i], blackPoints, m_cloudIDs[i], m_viewPorts[i]);
		}
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, m_cloudIDs[i]);
		viewer->createViewPortCamera(m_viewPorts[i]);
	}
	viewer->resetCamera();
}
void PCLViewer::updateLoop()
{

	std::chrono::milliseconds dura(100);
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer(m_viewerName));

	createViewPortsForViewer(viewer);

	{
		std::unique_lock<std::mutex> lock(m_cloudMutex);
		std::chrono::milliseconds dura(100);

		while (!m_cloudUpdate.wait_for(lock, dura) && !m_cloudsUpdated){
			if (viewer->wasStopped()){ return; }
			viewer->spinOnce();
		} 
		if (!m_isRunning){
			return;
		}
	}
	matchPointCloudsToViewPorts(viewer);
	
	while (!viewer->wasStopped() && m_isRunning)
	{
		{
			std::unique_lock<std::mutex> lock(m_cloudMutex);
			
			while (!m_cloudUpdate.wait_for(lock, dura) && !m_cloudsUpdated){
				viewer->spinOnce();
				if (!m_isRunning || viewer->wasStopped()){ return; }
			}
			std::unique_lock<std::mutex> viewPortConfigLock(m_viewPortConfigurationChangedMutex);
			if (m_viewPortConfigurationChanged){
				viewer.reset(new pcl::visualization::PCLVisualizer(m_viewerName));
				createViewPortsForViewer(viewer);
				matchPointCloudsToViewPorts(viewer);
				m_viewPortConfigurationChanged = false;
			}
			viewPortConfigLock.unlock();
			for (int i = 0; i < m_cloudCount; i++){
				std::lock_guard<std::mutex> lock(m_useColoredCloudMutex);
				updateCurrentCloudWithIndexAndIdentifier(i, m_cloudIDs[i], viewer);
			}
			m_cloudsUpdated = false;
		}
		viewer->spinOnce(100);
	}
}

void PCLViewer::setNumOfClouds(int numOfClouds)
{
	std::unique_lock<std::mutex> cloudLock(m_cloudMutex);
	std::lock_guard<std::mutex> viePortConfigLock(m_viewPortConfigurationChangedMutex);
	if (m_cloudCount != numOfClouds){
		m_cloudCount = numOfClouds;
		m_coloredClouds.resize(numOfClouds);
		m_cloudIDs.resize(numOfClouds);
		m_viewPorts.resize(numOfClouds);
		m_nonColoredClouds.resize(numOfClouds);
		m_viewPortConfigurationChanged = true;
	}
}

void PCLViewer::updateColoredCloud(int cloudIndex, std::string cloudID, pcl::visualization::PCLVisualizer::Ptr viewer)
{
	auto& cloud = m_coloredClouds[cloudIndex];
	if (cloud){
		viewer->updatePointCloud(cloud, cloudID);
	}
}

void PCLViewer::updateNonColoredCloud(int cloudIndex, std::string cloudID, pcl::visualization::PCLVisualizer::Ptr viewer)
{
	auto& cloud = m_nonColoredClouds[cloudIndex];
	if (cloud){
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> blackPoints(cloud, 0, 0, 0);
		viewer->updatePointCloud(cloud, blackPoints, cloudID);
		//viewer->updatePointCloud(cloud, cloudID);
	}
}

void PCLViewer::useColoredCloud(bool useColored)
{
	std::lock_guard<std::mutex> lock(m_useColoredCloudMutex);
	m_useColoredCloud = useColored;
	updateCurrentCloudWithIndexAndIdentifier.disconnect_all_slots();
	if (useColored){
		updateCurrentCloudWithIndexAndIdentifier.connect(boost::bind(&PCLViewer::updateColoredCloud, this, _1, _2, _3));
	}
	else{
		updateCurrentCloudWithIndexAndIdentifier.connect(boost::bind(&PCLViewer::updateNonColoredCloud, this, _1, _2, _3));
	}
}

