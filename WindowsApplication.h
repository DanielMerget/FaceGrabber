#pragma once
#include "stdafx.h"
#include <strsafe.h>
#include "resource.h"
#include "KinectHDFaceGrabber.h"
#include "resource.h"
#include "ImageRenderer.h"
#include "PCLViewer.h"
#include <memory>
#include "KinectCloudOutputWriter.h"
#include "KinectDepthGrabber.h"
#include <pcl/visualization/image_viewer.h>

class WindowsApplication
{
public:


	static const int       cColorWidth = 1920;
	static const int       cColorHeight = 1080;


	WindowsApplication();
	~WindowsApplication();

	void imageUpdated(const unsigned char *data, unsigned width, unsigned height);
	/// <summary>
	/// Handles window messages, passes most to the class instance to handle
	/// </summary>
	/// <param name="hWnd">window message is for</param>
	/// <param name="uMsg">message</param>
	/// <param name="wParam">message data</param>
	/// <param name="lParam">additional message data</param>
	/// <returns>result of message processing</returns>
	static LRESULT CALLBACK	MessageRouter(HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam);

	/// <summary>
	/// Handle windows messages for a class instance
	/// </summary>
	/// <param name="hWnd">window message is for</param>
	/// <param name="uMsg">message</param>
	/// <param name="wParam">message data</param>
	/// <param name="lParam">additional message data</param>
	/// <returns>result of message processing</returns>
	LRESULT CALLBACK		DlgProc(HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam);

	int						run(HINSTANCE hInstance, int nCmdShow);

	void processUIMessage(WPARAM wParam, LPARAM);

	bool					setStatusMessage(std::wstring statusString, bool bForce);

	HWND					m_hWnd;
	INT64					m_nStartTime;
	INT64					m_nLastCounter;
	double					m_fFreq;
	ULONGLONG				m_nNextStatusTime;
	DWORD					m_nFramesSinceUpdate;

	// Direct2D
	ImageRenderer*         m_pDrawDataStreams;
	ID2D1Factory*          m_pD2DFactory;
	

	KinectHDFaceGrabber m_kinectFrameGrabber;
	KinectDepthGrabber m_kinectDepthGrabber;
	std::shared_ptr<PCLViewer>	m_pclViewer;
	std::shared_ptr<pcl::visualization::ImageViewer> m_imageViewer;
	std::shared_ptr<KinectCloudOutputWriter> m_cloudOutputWriter;
};

