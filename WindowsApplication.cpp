#include "WindowsApplication.h"
#include <Commdlg.h>

WindowsApplication::WindowsApplication() :
	m_hWnd(NULL),
	m_nStartTime(0),
	m_nLastCounter(0),
	m_nFramesSinceUpdate(0),
	m_fFreq(0),
	m_nNextStatusTime(0),
	m_isCloudWritingStarted(false),
	m_recordingConfiguration(RECORD_CLOUD_TYPE_COUNT),
	m_isKinectRunning(true)
{
	LARGE_INTEGER qpf = { 0 };
	if (QueryPerformanceFrequency(&qpf))
	{
		m_fFreq = double(qpf.QuadPart);
	}
	initRecordDataModel();
}

void WindowsApplication::initRecordDataModel()
{
	for (int i = 0; i < RECORD_CLOUD_TYPE_COUNT; i++){
		m_recordingConfiguration[i] = std::shared_ptr<RecordingConfiguration>(new RecordingConfiguration(static_cast<RecordCloudType>(i), PLY));
		m_recordingConfiguration[i]->recordConfigurationStatusChanged.connect(boost::bind(&RecordTabHandler::recordConfigurationStatusChanged, &m_recordTabHandler, _1, _2));
		m_recordingConfiguration[i]->recordPathOrFileNameChanged.connect(boost::bind(&RecordTabHandler::recordPathChanged, &m_recordTabHandler, _1));
	}
	
	
	//m_recordingConfiguration[	HDFace	 ]->recordConfigurationStatusChanged.connect(boost::bind(&RecordTabHandler::recordConfigurationStatusChanged, &m_recordTabHandler, _1, _2));
	//m_recordingConfiguration[	FaceRaw	 ]->recordConfigurationStatusChanged.connect(boost::bind(&RecordTabHandler::recordConfigurationStatusChanged, &m_recordTabHandler, _1, _2));
	//m_recordingConfiguration[FullDepthRaw]->recordConfigurationStatusChanged.connect(boost::bind(&RecordTabHandler::recordConfigurationStatusChanged, &m_recordTabHandler, _1, _2));
	//
	//
	//m_recordingConfiguration[	HDFace	 ]->recordPathOrFileNameChanged.connect(boost::bind(&RecordTabHandler::recordPathChanged, &m_recordTabHandler, _1));
	//m_recordingConfiguration[	FaceRaw	 ]->recordPathOrFileNameChanged.connect(boost::bind(&RecordTabHandler::recordPathChanged, &m_recordTabHandler, _1));	
	//m_recordingConfiguration[FullDepthRaw]->recordPathOrFileNameChanged.connect(boost::bind(&RecordTabHandler::recordPathChanged, &m_recordTabHandler, _1));

	
	

	
}


WindowsApplication::~WindowsApplication()
{
	SafeRelease(m_pD2DFactory);
	m_bufferSynchronizer.onApplicationQuit();
	m_bufferSynchronizerThread.join();
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
		MAKEINTRESOURCE(IDD_APP_TAB),
		NULL,
		(DLGPROC)WindowsApplication::MessageRouter,
		reinterpret_cast<LPARAM>(this));

	// Show window
	ShowWindow(hWndApp, nCmdShow);

	// Main message loop
	while (WM_QUIT != msg.message)
	{
		if (m_isKinectRunning){
			m_kinectFrameGrabber.update();
		}
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

#include <windowsx.h>

int InsertTabItem(HWND hTab, LPTSTR pszText, int iid)
{
	TCITEM ti = { 0 };
	ti.mask = TCIF_TEXT;
	ti.pszText = pszText;
	ti.cchTextMax = wcslen(pszText);
	return TabCtrl_InsertItem(hTab, iid, &ti);
}

void WindowsApplication::disconnectWriterAndViewerToKinect()
{
	for (int i = 0; i < RECORD_CLOUD_TYPE_COUNT; i++){
		m_nonColoredOutputStreamUpdater->cloudUpdated[i].disconnect_all_slots();
		m_colouredOutputStreamUpdater->cloudUpdated[i].disconnect_all_slots();
	}
	m_nonColoredOutputStreamUpdater->cloudsUpdated.disconnect_all_slots();
	m_colouredOutputStreamUpdater->cloudsUpdated.disconnect_all_slots();
	m_colorCloudOutputWriter.clear();
	m_nonColoredCloudOutputWriter.clear();
}


void WindowsApplication::connectWriterAndViewerToKinect()
{
	for (int i = 0; i < RECORD_CLOUD_TYPE_COUNT; i++){
		auto nonColoredCloudWriter = std::shared_ptr<KinectCloudOutputWriter<pcl::PointXYZ>>(new KinectCloudOutputWriter<pcl::PointXYZ>);
		nonColoredCloudWriter->updateStatus.connect(boost::bind(&RecordTabHandler::updateWriterStatus, &m_recordTabHandler, static_cast<RecordCloudType>(i), _1));
		nonColoredCloudWriter->writingWasStopped.connect(boost::bind(&RecordTabHandler::recordingStopped, &m_recordTabHandler));

		m_nonColoredCloudOutputWriter.push_back(nonColoredCloudWriter);

		auto coloredCloudWriter = std::shared_ptr<KinectCloudOutputWriter<pcl::PointXYZRGB>>(new KinectCloudOutputWriter<pcl::PointXYZRGB>);
		coloredCloudWriter->updateStatus.connect(boost::bind(&RecordTabHandler::updateWriterStatus, &m_recordTabHandler, static_cast<RecordCloudType>(i), _1));

		coloredCloudWriter->writingWasStopped.connect(boost::bind(&RecordTabHandler::recordingStopped, &m_recordTabHandler));
		m_colorCloudOutputWriter.push_back(coloredCloudWriter);


		//we skip enabling the fulldepth raw
		if (i == FullDepthRaw){
			continue;
		}
		m_nonColoredOutputStreamUpdater->cloudUpdated[i].connect(boost::bind(&KinectCloudOutputWriter<pcl::PointXYZ>::updateCloudThreated, nonColoredCloudWriter, _1));
		m_colouredOutputStreamUpdater->cloudUpdated[i].connect(boost::bind(&KinectCloudOutputWriter<pcl::PointXYZRGB>::updateCloudThreated, coloredCloudWriter, _1));
	}
	m_nonColoredOutputStreamUpdater->cloudsUpdated.connect(boost::bind(&PCLViewer::updateNonColoredClouds, m_pclFaceViewer, _1));
	m_colouredOutputStreamUpdater->cloudsUpdated.connect(boost::bind(&PCLViewer::updateColoredClouds, m_pclFaceViewer, _1));
}

void WindowsApplication::onCreate()
{

	// Init Direct2D
	D2D1CreateFactory(D2D1_FACTORY_TYPE_SINGLE_THREADED, &m_pD2DFactory);

	// Create and initialize a new Direct2D image renderer (take a look at ImageRenderer.h)
	// We'll use this to draw the data we receive from the Kinect to the screen
	m_pDrawDataStreams = new ImageRenderer();
	m_pclFaceViewer = std::shared_ptr<PCLViewer>(new PCLViewer(2, "Face-Viewer"));

	m_recordTabHandler.setSharedRecordingConfiguration(m_recordingConfiguration);
	m_recordTabHandle = CreateDialogParamW(
		NULL,
		MAKEINTRESOURCE(IDC_TAB_RECORD),
		m_hWnd,
		(DLGPROC)RecordTabHandler::MessageRouterTab,
		reinterpret_cast<LPARAM>(&m_recordTabHandler));
	m_recordTabHandler.colorConfigurationChanged.connect(boost::bind(&WindowsApplication::colorStreamingChangedTo, this, _1));
	m_recordTabHandler.startWriting.connect(boost::bind(&WindowsApplication::startRecording, this, _1));
	m_recordTabHandler.stopWriting.connect(boost::bind(&WindowsApplication::stopRecording, this, _1));

	m_plackBackTabHandler.setSharedRecordingConfiguration(m_recordingConfiguration);

	m_playbackTabHandle = CreateDialogParamW(
		NULL,
		MAKEINTRESOURCE(IDC_TAB_PLAYBACK),
		m_hWnd,
		(DLGPROC)PlaybackTabHandler::MessageRouterTab,
		reinterpret_cast<LPARAM>(&m_plackBackTabHandler));
	ShowWindow(m_playbackTabHandle, SW_HIDE);
	m_plackBackTabHandler.startPlayback.connect(boost::bind(&WindowsApplication::startPlayback, this, _1, _2));
	m_plackBackTabHandler.stopPlayback.connect(boost::bind(&WindowsApplication::stopPlayback, this));

	m_convertTabHandle = CreateDialogParamW(
		NULL,
		MAKEINTRESOURCE(IDC_TAB_CONVERT),
		m_hWnd,
		(DLGPROC)ConvertTabHandler::MessageRouterTab,
		reinterpret_cast<LPARAM>(&m_convertTabHandler));
	ShowWindow(m_convertTabHandle, SW_HIDE);

	RECT windowRect;

	GetClientRect(m_hWnd, &windowRect);
	HWND tabControlHandle = GetDlgItem(m_hWnd, IDC_TAB2);

	InsertTabItem(tabControlHandle, L"Record", 0);
	InsertTabItem(tabControlHandle, L"Playback", 1);
	InsertTabItem(tabControlHandle, L"Convert", 2);
	//TabCtrl_InsertItem(m_hWnd, 0, &tab1Data);

	TabCtrl_SetCurSel(tabControlHandle, 0);
	ShowWindow(tabControlHandle, SW_SHOW);

	RECT tabControlRect;
	GetWindowRect(tabControlHandle, &tabControlRect);


	const int width = 869;
	const int height = 489;
	const int xPos = (windowRect.right - windowRect.left - width) / 2;
	m_liveViewWindow = CreateWindow(WC_STATIC, L"", WS_CHILD | WS_VISIBLE, xPos, tabControlRect.top, width, height, m_hWnd, NULL, m_hInstance, NULL);

	RECT liveViewRect;
	GetWindowRect(m_liveViewWindow, &liveViewRect);


	HRESULT hr = m_pDrawDataStreams->initialize(m_liveViewWindow, m_pD2DFactory, cColorWidth, cColorHeight, cColorWidth * sizeof(RGBQUAD));

	if (FAILED(hr))
	{
		setStatusMessage(L"Failed to initialize the Direct2D draw device.", true);
	}

	m_kinectFrameGrabber.setImageRenderer(m_pDrawDataStreams);

	// Get and initialize the default Kinect sensor
	m_kinectFrameGrabber.initializeDefaultSensor();
	int depthWidth, depthHeight, colorWidth, colorHeight;

	m_kinectFrameGrabber.getColourAndDepthSize(depthWidth, depthHeight, colorWidth, colorHeight);



	m_colouredOutputStreamUpdater = std::shared_ptr<ColouredOutputStreamUpdater>(new ColouredOutputStreamUpdater);
	m_nonColoredOutputStreamUpdater = std::shared_ptr<NonColouredOutputStreamsUpdater>(new NonColouredOutputStreamsUpdater);

	m_kinectFrameGrabber.setOutputStreamUpdater(m_colouredOutputStreamUpdater);

	m_colouredOutputStreamUpdater->initialize(m_kinectFrameGrabber.getCoordinateMapper(), depthWidth, depthHeight, colorWidth, colorHeight);
	m_nonColoredOutputStreamUpdater->initialize(m_kinectFrameGrabber.getCoordinateMapper(), depthWidth, depthHeight, colorWidth, colorHeight);

	connectWriterAndViewerToKinect();
	m_kinectFrameGrabber.statusChanged.connect(boost::bind(&WindowsApplication::setStatusMessage, this, _1, _2));
	std::vector<std::shared_ptr<Buffer<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>>> buffers;
	for (int i = 0; i < RECORD_CLOUD_TYPE_COUNT; i++){
		auto buffer = std::shared_ptr<Buffer<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>>(new Buffer<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>);
		auto inputReader = std::shared_ptr<PCLInputReader<pcl::PointXYZRGB>>(new PCLInputReader<pcl::PointXYZRGB>());
		inputReader->updateStatus.connect(boost::bind(&PlaybackTabHandler::updateReaderStatus, &m_plackBackTabHandler, static_cast<RecordCloudType>(i), _1));
		inputReader->setBuffer(buffer);

		m_bufferSynchronizer.updateStatus.connect(boost::bind(&PlaybackTabHandler::updateReaderStatus, &m_plackBackTabHandler, static_cast<RecordCloudType>(i), _1));

		m_inputFileReader.push_back(inputReader);
		buffers.push_back(buffer);
	}
	m_bufferSynchronizer.playbackFinished.connect(boost::bind(&WindowsApplication::onPlaybackFinished, this));
	m_bufferSynchronizerThread = std::thread(&BufferSynchronizer<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>::updateThreadFunc, &m_bufferSynchronizer);
	//m_bufferSynchronizer.setBuffer(buffers);
}

LRESULT CALLBACK WindowsApplication::DlgProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam)
{
	UNREFERENCED_PARAMETER(wParam);
	UNREFERENCED_PARAMETER(lParam);

	switch (message)
	{
	case WM_INITDIALOG:
		// Bind application window handle
		m_hWnd = hWnd;
		onCreate();
		break;

		// If the titlebar X is clicked, destroy app
	case WM_CLOSE:
		DestroyWindow(hWnd);
		m_pclFaceViewer->stopViewer();
		break;

	case WM_DESTROY:
		// Quit the main message pump
		//m_listView.OnDestroy(m_hWnd);
		PostQuitMessage(0);
		break;
	case WM_COMMAND:
		processUIMessage(wParam, lParam);
		break;
	case WM_NOTIFY:
		switch (((LPNMHDR)lParam)->code)
		{
		case TCN_SELCHANGE:
			int iPage = TabCtrl_GetCurSel(GetDlgItem(m_hWnd, IDC_TAB2));
			if (iPage == 0){
				if (m_plackBackTabHandler.isPlaybackRunning()){
					TabCtrl_SetCurSel(GetDlgItem(m_hWnd, IDC_TAB2), 1);
				}
				else{
					onRecordTabSelected();
				}
				
			}
			else if(iPage == 1){
				if (m_recordTabHandler.isRecording()){
					TabCtrl_SetCurSel(GetDlgItem(m_hWnd, IDC_TAB2), 0);
				}
				else{
					onPlaybackSelected();
				}
				
			}else if (iPage == 2){
				if (m_recordTabHandler.isRecording()){
					TabCtrl_SetCurSel(GetDlgItem(m_hWnd, IDC_TAB2), 0);
				}
				else if (m_plackBackTabHandler.isPlaybackRunning()){
					TabCtrl_SetCurSel(GetDlgItem(m_hWnd, IDC_TAB2), 1);
				}
				else{
					onConvertTabSelected();
				}
			}

			break;
		}
	
		default:
			break;
		
		break;

	}
	return FALSE;
}

void WindowsApplication::connectInputReaderToViewer()
{
	m_bufferSynchronizer.cloudsUpdated.connect(boost::bind(&PCLViewer::updateColoredClouds, m_pclFaceViewer, _1));
}

void WindowsApplication::disconnectInputReaderFromViewer()
{
	for (int i = 0; i < RECORD_CLOUD_TYPE_COUNT; i++){
		m_inputFileReader[i]->cloudUpdated.disconnect_all_slots();
	}
}

void WindowsApplication::onRecordTabSelected()
{
	m_isKinectRunning = true;
	m_plackBackTabHandler.playbackStopped();
	disconnectInputReaderFromViewer();
	connectWriterAndViewerToKinect();
	m_pclFaceViewer->useColoredCloud(m_recordTabHandler.isColorEnabled());
	m_pclFaceViewer->setNumOfClouds(2);
	ShowWindow(m_liveViewWindow, SW_SHOW);
	ShowWindow(m_recordTabHandle, SW_SHOW);
	ShowWindow(m_playbackTabHandle, SW_HIDE);
	ShowWindow(m_convertTabHandle, SW_HIDE);
}


//new
void WindowsApplication::onConvertTabSelected()
{
	ShowWindow(m_liveViewWindow, SW_HIDE);
	ShowWindow(m_recordTabHandle, SW_HIDE);
	ShowWindow(m_playbackTabHandle, SW_HIDE);
	ShowWindow(m_convertTabHandle, SW_SHOW);
}

void WindowsApplication::onPlaybackSelected()
{
	setStatusMessage(L"", true);
	m_isKinectRunning = false;
	disconnectWriterAndViewerToKinect();
	connectInputReaderToViewer();
	m_pclFaceViewer->useColoredCloud(true);
	ShowWindow(m_liveViewWindow, SW_HIDE);
	m_plackBackTabHandler.resetUIElements();
	m_plackBackTabHandler.setSharedRecordingConfiguration(m_recordTabHandler.getRecordConfiguration());
	ShowWindow(m_recordTabHandle, SW_HIDE);
	ShowWindow(m_playbackTabHandle, SW_SHOW);
	ShowWindow(m_convertTabHandle, SW_HIDE);
}


void WindowsApplication::startRecording(bool isColoredStream)
{
	
	int numEnabledCloudWriters = 0;
	for (int i = 0; i < RECORD_CLOUD_TYPE_COUNT; i++){
		auto recordingConfig = m_recordingConfiguration[i];
		if (recordingConfig->isEnabled()){
			numEnabledCloudWriters++;
		}
		
	}
	int numOfThreadsToStartPerWriter = 5 / numEnabledCloudWriters;
	numOfThreadsToStartPerWriter = std::max(numOfThreadsToStartPerWriter, 1);

	if (isColoredStream){
		
		for (int i = 0; i < RECORD_CLOUD_TYPE_COUNT; i++){
			auto recordingConfig = m_recordingConfiguration[i];
			auto cloudWriter = m_colorCloudOutputWriter[i];
			if (i == FullDepthRaw){
				m_colouredOutputStreamUpdater->cloudUpdated[i].disconnect_all_slots();
				m_nonColoredOutputStreamUpdater->cloudUpdated[i].disconnect_all_slots();
			}
			
			
			cloudWriter->setRecordingConfiguration(recordingConfig);
			if (recordingConfig->isEnabled()){
				if (i == FullDepthRaw){
					m_colouredOutputStreamUpdater->cloudUpdated[i].connect(boost::bind(&KinectCloudOutputWriter<pcl::PointXYZRGB>::updateCloudThreated, cloudWriter, _1));
				}
				cloudWriter->startWritingClouds(numOfThreadsToStartPerWriter);
			}
		}
	}
	else{

		for (int i = 0; i < RECORD_CLOUD_TYPE_COUNT; i++){
			auto recordingConfig = m_recordingConfiguration[i];
			auto cloudWriter = m_nonColoredCloudOutputWriter[i];
			cloudWriter->setRecordingConfiguration(recordingConfig);
			if (i == FullDepthRaw){
				m_colouredOutputStreamUpdater->cloudUpdated[i].disconnect_all_slots();
				m_nonColoredOutputStreamUpdater->cloudUpdated[i].disconnect_all_slots();
			}
			if (recordingConfig->isEnabled()){
				if (i == FullDepthRaw){
					m_nonColoredOutputStreamUpdater->cloudUpdated[i].connect(boost::bind(&KinectCloudOutputWriter<pcl::PointXYZ>::updateCloudThreated, cloudWriter, _1));
				}
				cloudWriter->startWritingClouds(numOfThreadsToStartPerWriter);
			}
		}
	}
}


void WindowsApplication::triggerReaderStart(SharedPlaybackConfiguration playbackConfig, bool isSingleThreatedReading)
{
	for (int i = 0; i < RECORD_CLOUD_TYPE_COUNT; i++){
		auto& currentConfig = playbackConfig[i];
		auto cloudType = currentConfig->getRecordCloudType();
		m_inputFileReader[cloudType]->setPlaybackConfiguration(playbackConfig[i]);
		m_inputFileReader[cloudType]->startCloudUpdateThread(isSingleThreatedReading);
	}
}
void WindowsApplication::setupReaderAndBuffersForPlayback(SharedPlaybackConfiguration playbackConfig)
{
	int enabledClouds = 0;
	std::vector<std::shared_ptr<Buffer<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>>> activeBuffers;
	int numOfFilesToRead = 0;
	for (int i = 0; i < RECORD_CLOUD_TYPE_COUNT; i++){
		auto& currentConfig = playbackConfig[i];
		if (currentConfig->isEnabled()){
			enabledClouds++;
			numOfFilesToRead = currentConfig->getCloudFilesToPlay().size();
			activeBuffers.push_back(m_inputFileReader[i]->getBuffer());
		}
	}

	m_bufferSynchronizer.setBuffer(activeBuffers, numOfFilesToRead);
	m_pclFaceViewer->setNumOfClouds(enabledClouds);
}
void WindowsApplication::startPlayback(SharedPlaybackConfiguration playbackConfig, bool isSingleThreatedReading)
{
	setupReaderAndBuffersForPlayback(playbackConfig);
	if (isSingleThreatedReading){
		std::async(std::launch::async, &WindowsApplication::triggerReaderStart, this, playbackConfig, isSingleThreatedReading);
	}
	else{
		triggerReaderStart(playbackConfig, isSingleThreatedReading);
	}
}

void WindowsApplication::onPlaybackFinished()
{
	m_plackBackTabHandler.playbackStopped();
}

void WindowsApplication::stopPlayback()
{
	for (auto& inputReader : m_inputFileReader){
		inputReader->stopReaderThreads();
	}
}

void WindowsApplication::stopRecording(bool isColoredStream)
{
	if (isColoredStream){
		for (int i = 0; i < RECORD_CLOUD_TYPE_COUNT; i++){
			auto recordingConfig = m_recordingConfiguration[i];
			auto cloudWriter = m_colorCloudOutputWriter[i];
			if (recordingConfig->isEnabled()){
				cloudWriter->stopWritingClouds();
			}
			if (i == FullDepthRaw){
				m_colouredOutputStreamUpdater->cloudUpdated[i].disconnect_all_slots();
			}
		}
	}
	else{
		for (int i = 0; i < RECORD_CLOUD_TYPE_COUNT; i++){
			auto recordingConfig = m_recordingConfiguration[i];
			auto cloudWriter = m_nonColoredCloudOutputWriter[i];
			if (recordingConfig->isEnabled()){
				cloudWriter->stopWritingClouds();
			}
			if (i == FullDepthRaw){
				m_nonColoredOutputStreamUpdater->cloudUpdated[i].disconnect_all_slots();
			}
		}
	}
}


void WindowsApplication::colorStreamingChangedTo(bool enable)
{
	if (enable){
		m_kinectFrameGrabber.setOutputStreamUpdater(m_colouredOutputStreamUpdater);
	}
	else{
		m_kinectFrameGrabber.setOutputStreamUpdater(m_nonColoredOutputStreamUpdater);
	}
	m_pclFaceViewer->useColoredCloud(enable);
}



void WindowsApplication::onSelectionChanged(WPARAM wParam, LPARAM handle)
{

	switch (LOWORD(wParam))
	{
	default:
		break;
	}
}
void WindowsApplication::onEditBoxeChanged(WPARAM wParam, LPARAM handle)
{

	switch (LOWORD(wParam))
	{
	default:
		break;
	}
}

void WindowsApplication::onButtonClicked(WPARAM wParam, LPARAM handle)
{
	switch (LOWORD(wParam))
	{
	default:
		break;
	}
}
void WindowsApplication::processUIMessage(WPARAM wParam, LPARAM handle)
{

	switch (HIWORD(wParam))
	{
	case CBN_SELCHANGE:
		onSelectionChanged(wParam, handle);
		break;
	case BN_CLICKED:
		onButtonClicked(wParam, handle);
		break;
	case EN_CHANGE:
		onEditBoxeChanged(wParam, handle);
	default:
		break;
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


void WindowsApplication::recordPathChanged(RecordCloudType type)
{

}

void WindowsApplication::recordConfigurationStatusChanged(RecordCloudType type, bool newState)
{
	
}




bool WindowsApplication::openFileDialog(WCHAR* szDir, HWND handle)
{
	OPENFILENAME ofn;
	ZeroMemory(&ofn, sizeof(ofn));

	// Initialize OPENFILENAME
	ZeroMemory(&ofn, sizeof(ofn));
	ofn.lStructSize = sizeof(ofn);
	ofn.hwndOwner = handle;
	ofn.lpstrFile = szDir;

	// Set lpstrFile[0] to '\0' so that GetOpenFileName does not
	// use the contents of szFile to initialize itself.
	ofn.lpstrFile[0] = '\0';
	ofn.nMaxFile = MAX_PATH;
	ofn.lpstrFilter = L"ply\0*.ply\0pcd\0*.pcd\0";
	ofn.nFilterIndex = 1;
	ofn.lpstrFileTitle = NULL;
	ofn.nMaxFileTitle = 0;
	ofn.lpstrInitialDir = NULL;
	ofn.Flags = OFN_PATHMUSTEXIST | OFN_FILEMUSTEXIST;

	if (GetOpenFileName(&ofn))
	{
		// Do something usefull with the filename stored in szFileName 
		return true;
	}
	return false;
}

bool WindowsApplication::openDirectoryDialog(WCHAR* szDir, HWND handle)
{
	BROWSEINFO bInfo;
	bInfo.hwndOwner = handle;
	bInfo.pidlRoot = NULL;
	bInfo.pszDisplayName = szDir; // Address of a buffer to receive the display name of the folder selected by the user
	bInfo.lpszTitle = L"Please, select a output folder"; // Title of the dialog
	bInfo.ulFlags = BIF_USENEWUI;
	bInfo.lpfn = NULL;
	bInfo.lParam = 0;
	bInfo.iImage = -1;

	LPITEMIDLIST lpItem = SHBrowseForFolder(&bInfo);
	if (lpItem != NULL)
	{
		if (SHGetPathFromIDList(lpItem, szDir)){
			return true;
		}
	}
	return false;
}