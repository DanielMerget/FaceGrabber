#include "WindowsApplication.h"


WindowsApplication::WindowsApplication() :
	m_hWnd(NULL),
	m_nStartTime(0),
	m_nLastCounter(0),
	m_nFramesSinceUpdate(0),
	m_fFreq(0),
	m_nNextStatusTime(0),
	m_isCloudWritingStarted(false)
	//m_cloudViewer("CloudViewer")
{
	LARGE_INTEGER qpf = { 0 };
	if (QueryPerformanceFrequency(&qpf))
	{
		m_fFreq = double(qpf.QuadPart);
	}
}


WindowsApplication::~WindowsApplication()
{
	SafeRelease(m_pD2DFactory);
}




/// <summary>
/// Creates the main window and begins processing
/// </summary>
/// <param name="hInstance">handle to the application instance</param>
/// <param name="nCmdShow">whether to display minimized, maximized, or normally</param>
int WindowsApplication::run(HINSTANCE hInstance, int nCmdShow)
{

	MSG       msg = { 0 };
	WNDCLASS  wc;

	// Dialog custom window class
	ZeroMemory(&wc, sizeof(wc));
	wc.style = CS_HREDRAW | CS_VREDRAW;
	wc.cbWndExtra = DLGWINDOWEXTRA;
	wc.hCursor = LoadCursorW(NULL, IDC_ARROW);
	wc.hIcon = LoadIconW(hInstance, MAKEINTRESOURCE(IDI_APP));
	wc.lpfnWndProc = DefDlgProcW;
	wc.lpszClassName = L"KinectHDFaceGrabberAppDlgWndClass";
	m_hInstance = hInstance;
	if (!RegisterClassW(&wc))
	{
		return 0;
	}

	// Create main application window
	HWND hWndApp = CreateDialogParamW(
		NULL,
		MAKEINTRESOURCE(IDD_APP),
		NULL,
		(DLGPROC)WindowsApplication::MessageRouter,
		reinterpret_cast<LPARAM>(this));

	// Show window
	ShowWindow(hWndApp, nCmdShow);

	// Main message loop
	while (WM_QUIT != msg.message)
	{
		m_kinectFrameGrabber.update();
		//m_kinectDepthGrabber.update();
		while (PeekMessageW(&msg, NULL, 0, 0, PM_REMOVE))
		{
			// If a dialog message will be taken care of by the dialog proc
			if (hWndApp && IsDialogMessageW(hWndApp, &msg))
			{
				continue;
			}

			TranslateMessage(&msg);
			DispatchMessageW(&msg);
		}
		//m_cloudViewer.wasStopped();
	}

	return static_cast<int>(msg.wParam);
}


/// <summary>
/// Handles window messages, passes most to the class instance to handle
/// </summary>
/// <param name="hWnd">window message is for</param>
/// <param name="uMsg">message</param>
/// <param name="wParam">message data</param>
/// <param name="lParam">additional message data</param>
/// <returns>result of message processing</returns>
LRESULT CALLBACK WindowsApplication::MessageRouter(HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam)
{
	WindowsApplication* pThis = nullptr;

	if (WM_INITDIALOG == uMsg)
	{
		pThis = reinterpret_cast<WindowsApplication*>(lParam);
		SetWindowLongPtr(hWnd, GWLP_USERDATA, reinterpret_cast<LONG_PTR>(pThis));
	}
	else
	{
		pThis = reinterpret_cast<WindowsApplication*>(::GetWindowLongPtr(hWnd, GWLP_USERDATA));
	}

	if (pThis)
	{
		return pThis->DlgProc(hWnd, uMsg, wParam, lParam);
	}

	return 0;
}

void WindowsApplication::imageUpdated(const unsigned char *data, unsigned width, unsigned height)
{
	OutputDebugString(L"Imageupdate");
	//this->m_imageViewer->showRGBImage(data, width, height);
}
void WindowsApplication::cloudUpdate(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
{
	//m_cloudViewer.showCloud(cloud);
}

LRESULT CALLBACK WindowsApplication::DlgProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam)
{
	UNREFERENCED_PARAMETER(wParam);
	UNREFERENCED_PARAMETER(lParam);

	switch (message)
	{
	case WM_INITDIALOG:
	{
		// Bind application window handle
		m_hWnd = hWnd;

		// Init Direct2D
		D2D1CreateFactory(D2D1_FACTORY_TYPE_SINGLE_THREADED, &m_pD2DFactory);

		// Create and initialize a new Direct2D image renderer (take a look at ImageRenderer.h)
		// We'll use this to draw the data we receive from the Kinect to the screen
		m_pDrawDataStreams = new ImageRenderer();
		m_pclFaceViewer = std::shared_ptr<PCLViewer>(new PCLViewer(2, "Face-Viewer"));
		//m_pclFaceRawViewer = std::shared_ptr<PCLViewer>(new PCLViewer("Raw Face-Depth"));
		
		m_cloudOutputWriter = std::shared_ptr<KinectCloudOutputWriter>(new KinectCloudOutputWriter);
		HRESULT hr = m_pDrawDataStreams->initialize(GetDlgItem(m_hWnd, IDC_VIDEOVIEW), m_pD2DFactory, cColorWidth, cColorHeight, cColorWidth * sizeof(RGBQUAD));
		
		//HRESULT hr = m_pDrawDataStreams->initialize(GetDlgItem(m_hWnd, IDC_TAB2), m_pD2DFactory, cColorWidth, cColorHeight, cColorWidth * sizeof(RGBQUAD));
		TCITEM tab1Data;
		tab1Data.mask = TCIF_TEXT;
		tab1Data.pszText = L"Tab1";

		TabCtrl_InsertItem(m_hWnd, 0, &tab1Data);
		
		TCITEM tab2Data;
		tab2Data.mask = TCIF_TEXT;
		tab2Data.pszText = L"Tab2";

		TabCtrl_InsertItem(m_hWnd, 1, &tab2Data);
		
		HWND tab1Handle = CreateWindow(WC_STATIC, L"blabla1", WS_CHILD, 6, 40, 474, 320, m_hWnd, NULL, m_hInstance, NULL);
		HWND tab2Handle = CreateWindow(WC_STATIC, L"blabla2", WS_CHILD, 6, 40, 474, 320, m_hWnd, NULL, m_hInstance, NULL);
		ShowWindow(tab1Handle, SW_SHOW);

		//HRESULT hr = m_pDrawDataStreams->initialize(tab1Handle, m_pD2DFactory, cColorWidth, cColorHeight, cColorWidth * sizeof(RGBQUAD));

		if (FAILED(hr))
		{
			setStatusMessage(L"Failed to initialize the Direct2D draw device.", true);
		}
		
		m_kinectFrameGrabber.setImageRenderer(m_pDrawDataStreams);
		
		// Get and initialize the default Kinect sensor
		m_kinectFrameGrabber.initializeDefaultSensor();
		//m_kinectFrameGrabber.depthCloudUpdated.connect(boost::bind(&WindowsApplication::cloudUpdate, this, _1));		

		m_kinectFrameGrabber.cloudUpdated.connect(boost::bind(&PCLViewer::updateCloudThreated, m_pclFaceViewer, _1, 0));
		m_kinectFrameGrabber.depthCloudUpdated.connect(boost::bind(&PCLViewer::updateCloudThreated, m_pclFaceViewer, _1, 1));

		m_kinectFrameGrabber.depthCloudUpdated.connect(boost::bind(&KinectCloudOutputWriter::updateCloudThreated, m_cloudOutputWriter, _1));
		//m_kinectFrameGrabber.cloudUpdated.connect(boost::bind(&KinectCloudOutputWriter::updateCloudThreated, m_cloudOutputWriter, _1));

		m_kinectFrameGrabber.statusChanged.connect(boost::bind(&WindowsApplication::setStatusMessage, this, _1, _2));

	}
		break;

		// If the titlebar X is clicked, destroy app
	case WM_CLOSE:
		DestroyWindow(hWnd);
		//m_pclFaceRawViewer->stopViewer();
		m_pclFaceViewer->stopViewer();
		break;

	case WM_DESTROY:
		// Quit the main message pump
		PostQuitMessage(0);
		break;
	case WM_COMMAND:
		processUIMessage(wParam, lParam);
		break;
	}

	return FALSE;
}
#include <iostream>
void WindowsApplication::processUIMessage(WPARAM wParam, LPARAM)
{
	if (IDC_RECORD_BUTTON == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam)){
		if (!m_isCloudWritingStarted)
		{
			m_cloudOutputWriter->startWritingClouds();
			SetDlgItemText(m_hWnd, IDC_RECORD_BUTTON, L"Stop");
		}
		else{
			SetDlgItemText(m_hWnd, IDC_RECORD_BUTTON, L"Record");
			m_cloudOutputWriter->stopWritingClouds();
		}
		m_isCloudWritingStarted = !m_isCloudWritingStarted;
	}
}

bool WindowsApplication::setStatusMessage(std::wstring statusString, bool bForce)
{
	_In_z_ WCHAR* szMessage = &statusString[0];
	ULONGLONG now = GetTickCount64();
	ULONGLONG nShowTimeMsec = 1000;
	if (m_hWnd && (bForce || (m_nNextStatusTime <= now)))
	{
		SetDlgItemText(m_hWnd, IDC_STATUS, szMessage);
		m_nNextStatusTime = now + nShowTimeMsec;

		return true;
	}

	return false;
}
