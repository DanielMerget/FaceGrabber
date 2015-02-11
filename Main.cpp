
#include "stdafx.h"
#include <strsafe.h>
#include "resource.h"
#include "KinectHDFaceGrabber.h"

#include "WindowsApplication.h"

#include "PCLInputReader.h"
#include "PCLViewer.h"
#include <pcl/io/ply_io.h>


int APIENTRY wWinMain(_In_ HINSTANCE hInstance, _In_opt_ HINSTANCE hPrevInstance, _In_ LPWSTR lpCmdLine, _In_ int nCmdShow)
{
	UNREFERENCED_PARAMETER(hPrevInstance);
	UNREFERENCED_PARAMETER(lpCmdLine);

	WindowsApplication application;
	application.run(hInstance, nCmdShow);
}


#include "stdafx.h"
#define NOMINMAX
#include <Windows.h>
#include <Kinect.h>
#include <pcl/visualization/cloud_viewer.h>


int main()
{
	
	std::cout << "main started" << std::endl;
	PCLInputReader reader("", "Cloud_", 99, 309);
	
	std::shared_ptr<PCLViewer> viewer(new PCLViewer());

	reader.cloudUpdated.connect(boost::bind(&PCLViewer::updateCloudThreated, viewer, _1 , 1));
	reader.startReaderThreads();
	
	
	reader.startCloudUpdateThread();
	reader.join();

}
