
#include "stdafx.h"
#include <strsafe.h>
#include "resource.h"
#include "KinectHDFaceGrabber.h"

#include "WindowsApplication.h"

#include "PCLInputReader.h"
#include "PCLViewer.h"
#include <pcl/io/ply_io.h>


/// <summary>
/// Entry point for the application
/// </summary>
/// <param name="hInstance">handle to the application instance</param>
/// <param name="hPrevInstance">always 0</param>
/// <param name="lpCmdLine">command line arguments</param>
/// <param name="nCmdShow">whether to display minimized, maximized, or normally</param>
/// <returns>status</returns>
int APIENTRY wWinMain(_In_ HINSTANCE hInstance, _In_opt_ HINSTANCE hPrevInstance, _In_ LPWSTR lpCmdLine, _In_ int nCmdShow)
{
	UNREFERENCED_PARAMETER(hPrevInstance);
	UNREFERENCED_PARAMETER(lpCmdLine);


	//PCLViewer viewer;
	//pcl::PCLPointCloud2 blob;

	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud <pcl::PointXYZRGB>());
	//pcl::io::loadPLYFile("myFace", blob);
	//pcl::fromPCLPointCloud2(blob, *cloud);

	//viewer.updateCloud(cloud);


	//PCLInputReader reader("", "Cloud_", 10, 90);

	//PCLViewer viewer;
	//reader.cloudUpdated.connect(boost::bind(&PCLViewer::updateCloudThreated, &viewer, _1));
	//reader.startReaderThreads();


	//reader.startCloudUpdateThread();

	WindowsApplication application;
	application.run(hInstance, nCmdShow);
//	reader.join();
}

pcl::visualization::CloudViewer viewer("Cloud Viewer");

void update(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
{
	std::cout << "update.." << std::endl;
	viewer.showCloud(cloud);
}



int main()
{
	////const std::string inputPath, const std::string fileNamePrefix, const int bufferSize, const int numOfFilesToRead);
	
	std::cout << "main started" << std::endl;
	PCLInputReader reader("", "Cloud_", 10, 90);
	
	//PCLViewer viewer;
	
	
	reader.cloudUpdated.connect(boost::bind(update, _1));
	reader.startReaderThreads();
	
	
	reader.startCloudUpdateThread();
	reader.join();
	while (!viewer.wasStopped())
	{
	}
	//PCLViewer viewer;
	//pcl::PCLPointCloud2 blob;

	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud <pcl::PointXYZRGB>());
	//pcl::io::loadPLYFile("Cloud_0.ply", blob);
	//pcl::fromPCLPointCloud2(blob, *cloud);

	//viewer.updateCloud(cloud);
}