#include "PCLViewer.h"
#include <future>

PCLViewer::PCLViewer() :
	m_updateThread(&PCLViewer::updateLoop, this)
{
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

void PCLViewer::updateCloudThreated(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
{
	std::async(&PCLViewer::updateCloud, this, cloud);
}

void PCLViewer::updateCloud(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
{
	std::unique_lock<std::mutex> lock(m_cloudMutex);
	m_currentCloud = cloud;
	m_cloudUpdate.notify_all();
}

void PCLViewer::updateLoop()
{
	m_isRunning = true;
	pcl::visualization::PCLVisualizer viewer("My new one");
	{
		std::unique_lock<std::mutex> lock(m_cloudMutex);
		//m_cloudUpdate.wait(lock);
		std::chrono::milliseconds dura(100);
		//std::this_thread::sleep_for(dura);

		while (!m_cloudUpdate.wait_for(lock, dura)){
			viewer.spinOnce();
		} 
			

		viewer.addPointCloud(m_currentCloud, "cloud");
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");
		viewer.initCameraParameters();
	}
	while (!viewer.wasStopped() && m_isRunning)
	{
		{
			std::unique_lock<std::mutex> lock(m_cloudMutex);
			m_cloudUpdate.wait(lock);
			viewer.updatePointCloud(m_currentCloud, "cloud");
		}
		viewer.spinOnce(100);
	}

}
