#include "PCLViewer.h"
#include <future>

PCLViewer::PCLViewer(std::string viewerName) :
	m_viewerName(viewerName),
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

void PCLViewer::updateCloudThreated(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud, int index)
{
	std::async(std::launch::async, &PCLViewer::updateCloud, this, cloud, index);
}

void PCLViewer::updateCloud(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud, int index)
{
	std::unique_lock<std::mutex> lock(m_cloudMutex);
	if (index == 1){
		m_currentCloud1 = cloud;
	}
	else{
		m_currentCloud2 = cloud;
	}
	if (m_currentCloud1 && m_currentCloud2){
		m_cloudUpdate.notify_all();
	}
}

void PCLViewer::updateLoop()
{
	m_isRunning = true;
	//static std::mutex constructorLock;
	//constructorLock.lock();
	pcl::visualization::PCLVisualizer viewer(m_viewerName);
	int viewPortID1;
	int viewPortID2;
	viewer.createViewPort(0, 0, 0.5, 1, viewPortID1);
	viewer.createViewPort(0.5, 0, 1, 1, viewPortID2);
	//constructorLock.unlock();
	{
		std::unique_lock<std::mutex> lock(m_cloudMutex);
		//m_cloudUpdate.wait(lock);
		std::chrono::milliseconds dura(100);
		//std::this_thread::sleep_for(dura);

		while (!m_cloudUpdate.wait_for(lock, dura)){
			viewer.spinOnce();
		} 

		viewer.addPointCloud(m_currentCloud1, "cloud1", viewPortID1);
		viewer.addPointCloud(m_currentCloud2, "cloud2", viewPortID2);
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud1");
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud2");
		viewer.createViewPortCamera(viewPortID1);
		viewer.createViewPortCamera(viewPortID2);
		viewer.resetCamera();
	}
	while (!viewer.wasStopped() && m_isRunning)
	{
		{
			std::unique_lock<std::mutex> lock(m_cloudMutex);
			//m_cloudUpdate.wait(lock);
			std::chrono::milliseconds dura(100);
			while (!m_cloudUpdate.wait_for(lock, dura)){
				viewer.spinOnce();
			}
			viewer.updatePointCloud(m_currentCloud1, "cloud1");
			viewer.updatePointCloud(m_currentCloud2, "cloud2");
		}
		viewer.spinOnce(100);
	}

}
