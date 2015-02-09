#include "PCLViewer.h"


PCLViewer::PCLViewer()
{
	m_updateThreads.push_back(std::thread(&PCLViewer::updateLoop, this));
}


PCLViewer::~PCLViewer()
{
	for (auto& thread : m_updateThreads){
		thread.join();
	}
}

bool PCLViewer::isStopped(){
	//return viewer.wasStopped();
	return false;
}

void PCLViewer::updateCloudThreated(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
{
	//m_updateThreads.push_back(std::thread(&PCLViewer::updateCloud, this, cloud));
	//updateCloud(cloud);
	std::unique_lock<std::mutex> lock(m_cloudMutex);
	//m_cloudUpdate.wait(lock);

	m_currentCloud = cloud;
	m_cloudUpdate.notify_all();
}

void PCLViewer::updateLoop()
{
	pcl::visualization::PCLVisualizer viewer("My new one");
	{
		std::unique_lock<std::mutex> lock(m_cloudMutex);
		m_cloudUpdate.wait(lock);

		viewer.addPointCloud(m_currentCloud, "cloud");
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "cloud");
		viewer.initCameraParameters();
		viewer.addCoordinateSystem(1.0);
	}
	while (!viewer.wasStopped())
	{
		{
			std::unique_lock<std::mutex> lock(m_cloudMutex);
			m_cloudUpdate.wait(lock);
			viewer.updatePointCloud(m_currentCloud, "cloud");
		}
		viewer.spinOnce(100);

		//std::chrono::milliseconds dura(100);
		//std::this_thread::sleep_for(dura);
	}

	/*if (first){	
		viewer.addPointCloud(cloud, "cloud");
		first = false;
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "cloud");
		viewer.initCameraParameters();
		viewer.addCoordinateSystem(1.0);
	}
	else{
		viewer.updatePointCloud(cloud, "cloud");
		viewer.spinOnce(100);
	}*/
}
