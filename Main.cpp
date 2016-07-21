#include "stdafx.h"
#include "WindowsApplication.h"


#include "resource.h"
//#include "KinectHDFaceGrabber.h"
//#include <tchar.h>
#include <strsafe.h>
//#include "PCLInputReader.h"
//#include "PCLViewer.h"
//#include <pcl/io/ply_io.h>


int APIENTRY wWinMain(_In_ HINSTANCE hInstance, _In_opt_ HINSTANCE hPrevInstance, _In_ LPWSTR lpCmdLine, _In_ int nCmdShow)
{
	UNREFERENCED_PARAMETER(hPrevInstance);
	UNREFERENCED_PARAMETER(lpCmdLine);
	//m_recordingConfiguration[i].recordConfigurationEnabled.connect(boost::bind(&WindowsApplication::recordConfigurationStatusChanged, this));
	
	WindowsApplication application;
	application.run(hInstance, nCmdShow);
}


//#include "stdafx.h"
//#include <Windows.h>
//#include <Kinect.h>
////#include <pcl/visualization/cloud_viewer.h>
//
//#include <regex>

//#include <pcl/io/pcd_grabber.h>
//#include <pcl/console/parse.h>
//#include <pcl/console/print.h>
//#include <pcl/visualization/boost.h>
//#include <pcl/visualization/cloud_viewer.h>
//#include <pcl/visualization/image_viewer.h>

/*
int main()
{
	std::smatch m;
	const std::string input = "Cloud_1234.ply";
	std::regex_search(input, m, std::regex("([a-z])*_(?)(.ply)"));
	std::cout << "prefix: " << m.prefix() << " suffix: " << m.suffix() << std::endl;

	//std::cout << "main started" << std::endl;
	//PCLInputReader reader("", "Cloud_", 99, 309);
	//
	//std::shared_ptr<PCLViewer> viewer(new PCLViewer(1, "Face-Playback"));
	//
	//reader.cloudUpdated.connect(boost::bind(&PCLViewer::updateColoredCloudThreated, viewer, _1, 0));
	//reader.startReaderThreads();
	//
	//
	//reader.startCloudUpdateThread();
	//reader.join();

}
*/