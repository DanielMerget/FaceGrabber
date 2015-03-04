#include "PCLViewer.h"
#include <future>

PCLViewer::PCLViewer(int cloudCount, std::string viewerName) :
	m_cloudCount(cloudCount),
	m_viewerName(viewerName),
	m_coloredClouds(cloudCount),
	m_nonColoredClouds(cloudCount),
	m_useColoredCloud(true),
	m_isRunning(false),
	m_updateThread(&PCLViewer::updateLoop, this)
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

void PCLViewer::updateNonColoredCloudThreated(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, int index)
{
	std::async(std::launch::async, &PCLViewer::pushNewNonColoredCloudAtIndex, this, cloud, index);
}
void PCLViewer::updateColoredCloudThreated(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud, int index)
{
	std::async(std::launch::async, &PCLViewer::pushNewColoredCloudAtIndex, this, cloud, index);
}

void PCLViewer::pushNewColoredCloudAtIndex(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud, int index)
{
	if (index >= m_cloudCount){
		return;
	}
	std::unique_lock<std::mutex> lock(m_cloudMutex);
	m_coloredClouds[index] = cloud;
	//if (index == 1){
	//	m_currentCloud1 = cloud;
	//}
	//else{
	//	m_currentCloud2 = cloud;
	//}
	//if (m_currentCloud1 && m_currentCloud2){
	//	m_cloudUpdate.notify_all();
	//}
	if (!m_isRunning){		
		for (auto& cloud : m_coloredClouds){
			if (!cloud){
				return;
			}
		}
		m_isRunning = true;
		m_cloudUpdate.notify_all();
	}
	else{
		m_cloudUpdate.notify_all();
	}
}

void PCLViewer::pushNewNonColoredCloudAtIndex(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, int index)
{
	if (index >= m_cloudCount){
		return;
	}
	std::unique_lock<std::mutex> lock(m_cloudMutex);
	m_nonColoredClouds[index] = cloud;
	//if (index == 1){
	//	m_currentCloud1 = cloud;
	//}
	//else{
	//	m_currentCloud2 = cloud;
	//}
	//if (m_currentCloud1 && m_currentCloud2){
	//	m_cloudUpdate.notify_all();
	//}
	if (!m_isRunning){
		for (auto& cloud : m_nonColoredClouds){
			if (!cloud){
				return;
			}
		}
		m_isRunning = true;
		m_cloudUpdate.notify_all();
	}
	else{
		m_cloudUpdate.notify_all();
	}
}

void PCLViewer::updateLoop()
{
	
	//static std::mutex constructorLock;
	//constructorLock.lock();
	pcl::visualization::PCLVisualizer viewer(m_viewerName);
	//int viewPortID1;
	//int viewPortID2;
	std::vector<int> viewPorts(m_cloudCount);
	std::vector<std::string> cloudIDs(m_cloudCount);
	float horizontalSplitPlaneWidth = 1.0f / m_cloudCount;
	for (int i = 0; i < m_cloudCount; i++){
		float xMin = horizontalSplitPlaneWidth * i;
		float xMax = horizontalSplitPlaneWidth * (i + 1);
		viewer.createViewPort(xMin, 0, xMax, 1.0, viewPorts[i]);
		cloudIDs[i] = std::to_string(i);
	}

	{
		std::unique_lock<std::mutex> lock(m_cloudMutex);
		std::chrono::milliseconds dura(100);

		while (!m_cloudUpdate.wait_for(lock, dura)){
			viewer.spinOnce();
		} 
		if (!m_isRunning){
			return;
		}
		for (int i = 0; i < m_cloudCount; i++){
			viewer.addPointCloud(m_coloredClouds[i], cloudIDs[i], viewPorts[i]);
			viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, cloudIDs[i]);
			viewer.createViewPortCamera(viewPorts[i]);
		}

		viewer.resetCamera();
	}
	while (!viewer.wasStopped() && m_isRunning)
	{
		{
			std::unique_lock<std::mutex> lock(m_cloudMutex);
			std::chrono::milliseconds dura(100);
			while (!m_cloudUpdate.wait_for(lock, dura)){
				viewer.spinOnce();
			}
			for (int i = 0; i < m_cloudCount; i++){
				std::lock_guard<std::mutex> lock(m_useColoredCloudMutex);
				updateCurrentCloudWithIndexAndIdentifier(i, cloudIDs[i], viewer);
			}
		}
		viewer.spinOnce(100);
	}

}

void PCLViewer::updateColoredCloud(int cloudIndex, std::string cloudID, pcl::visualization::PCLVisualizer& viewer)
{
	auto& cloud = m_coloredClouds[cloudIndex];
	if (cloud){
		viewer.updatePointCloud(cloud, cloudID);
	}
}

void PCLViewer::updateNonColoredCloud(int cloudIndex, std::string cloudID, pcl::visualization::PCLVisualizer& viewer)
{
	auto& cloud = m_nonColoredClouds[cloudIndex];
	if (cloud){
		viewer.updatePointCloud(cloud, cloudID);
	}
}

void PCLViewer::useColoredCloud(bool useColored)
{
	std::lock_guard<std::mutex> lock(m_useColoredCloudMutex);
	if (useColored){
		updateCurrentCloudWithIndexAndIdentifier.connect(boost::bind(&PCLViewer::updateColoredCloud, this, _1, _2, _3));
	}
	else{
		updateCurrentCloudWithIndexAndIdentifier.connect(boost::bind(&PCLViewer::updateNonColoredCloud, this, _1, _2, _3));
	}
}

