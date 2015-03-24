#include "stdafx.h"
#include "WindowsApplication.h"
#include <windowsx.h>

WindowsApplication::WindowsApplication() :
	m_nNextStatusTime(0),
	m_isKinectRunning(true),
	m_bufferSynchronizer(true),
	m_pDrawDataStreams(nullptr),
	m_pD2DFactory(nullptr)
{
	
}

SharedRecordingConfiguration WindowsApplication::initRecordDataModel()
{
	SharedRecordingConfiguration recordingConfigurations(RECORD_CLOUD_TYPE_COUNT);
	for (int i = 0; i < RECORD_CLOUD_TYPE_COUNT; i++){
		recordingConfigurations[i] = std::shared_ptr<RecordingConfiguration>(new RecordingConfiguration(static_cast<RecordCloudType>(i), PLY));
		recordingConfigurations[i]->recordConfigurationStatusChanged.connect(boost::bind(&RecordTabHandler::recordConfigurationStatusChanged, &m_recordTabHandler, _1, _2));
		recordingConfigurations[i]->recordPathOrFileNameChanged.connect(boost::bind(&RecordTabHandler::recordPathChanged, &m_recordTabHandler, _1));
		recordingConfigurations[i]->setThreadCountToStart(2);
	}
	return recordingConfigurations;
}


WindowsApplication::~WindowsApplication()
{
	SafeRelease(m_pD2DFactory);
	m_bufferSynchronizer.onApplicationQuit();
	m_bufferSynchronizerThread.join();
}

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
		(DLGPROC)MessageRouterHelper::MessageRouter,
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
	}

	return static_cast<int>(msg.wParam);
}

/**
 * \fn	void WindowsApplication::onCreate()
 *
 * \brief	Executes the create action.
 *
 * \author	Martin
 * \date	17.03.2015
 */

void WindowsApplication::onCreate()
{

	// Init Direct2D
	D2D1CreateFactory(D2D1_FACTORY_TYPE_SINGLE_THREADED, &m_pD2DFactory);

	// Create and initialize a new Direct2D image renderer (take a look at ImageRenderer.h)
	// We'll use this to draw the data we receive from the Kinect to the screen
	m_pDrawDataStreams = new ImageRenderer();
	m_pclFaceViewer = std::shared_ptr<PCLViewer>(new PCLViewer(2, "Face-Viewer"));

	
	initKinectFrameGrabber();
	initTabs();
	initCloudWriter();
	connectStreamUpdaterToViewer();

	initInputReaderBufferAndSynchronizer();

	HRESULT hr = m_pDrawDataStreams->initialize(m_liveViewWindow, m_pD2DFactory, cColorWidth, cColorHeight, cColorWidth * sizeof(RGBQUAD));

	if (FAILED(hr))
	{
		setStatusMessage(L"Failed to initialize the Direct2D draw device.", true);
	}

}

int WindowsApplication::insertTabItem(HWND tab, LPTSTR text, int tabid)
{
	TCITEM ti = { 0 };
	ti.mask = TCIF_TEXT;
	ti.pszText = text;
	ti.cchTextMax = wcslen(text);
	return TabCtrl_InsertItem(tab, tabid, &ti);
}

void WindowsApplication::disconnectStreamUpdaterFromViewer()
{
	for (int i = 0; i < RECORD_CLOUD_TYPE_COUNT; i++){
		m_nonColoredOutputStreamUpdater->cloudUpdated[i].disconnect_all_slots();
		m_colouredOutputStreamUpdater->cloudUpdated[i].disconnect_all_slots();
	}
	m_nonColoredOutputStreamUpdater->cloudsUpdated.disconnect_all_slots();
	m_colouredOutputStreamUpdater->cloudsUpdated.disconnect_all_slots();
	//m_colorCloudOutputWriter.clear();
	//m_nonColoredCloudOutputWriter.clear();
}


void WindowsApplication::initCloudWriter()
{
	for (int i = 0; i < RECORD_CLOUD_TYPE_COUNT; i++){
		auto nonColoredCloudWriter = std::shared_ptr<KinectCloudFileWriter<pcl::PointXYZ>>(new KinectCloudFileWriter<pcl::PointXYZ>);
		nonColoredCloudWriter->updateStatus.connect(boost::bind(&RecordTabHandler::updateWriterStatus, &m_recordTabHandler, static_cast<RecordCloudType>(i), _1));
		nonColoredCloudWriter->writingWasStopped.connect(boost::bind(&RecordTabHandler::recordingStopped, &m_recordTabHandler));

		m_nonColoredCloudOutputWriter.push_back(nonColoredCloudWriter);

		auto coloredCloudWriter = std::shared_ptr<KinectCloudFileWriter<pcl::PointXYZRGB>>(new KinectCloudFileWriter<pcl::PointXYZRGB>);
		coloredCloudWriter->updateStatus.connect(boost::bind(&RecordTabHandler::updateWriterStatus, &m_recordTabHandler, static_cast<RecordCloudType>(i), _1));

		coloredCloudWriter->writingWasStopped.connect(boost::bind(&RecordTabHandler::recordingStopped, &m_recordTabHandler));
		m_colorCloudOutputWriter.push_back(coloredCloudWriter);
	}
}

void WindowsApplication::connectStreamUpdaterToViewer()
{
	for (int i = 0; i < RECORD_CLOUD_TYPE_COUNT; i++){
		//we skip enabling the fulldepth raw
		if (i == FullDepthRaw){
			continue;
		}
		m_nonColoredOutputStreamUpdater->cloudUpdated[i].connect(boost::bind(&KinectCloudFileWriter<pcl::PointXYZ>::pushCloudAsync, m_nonColoredCloudOutputWriter[i], _1));
		m_colouredOutputStreamUpdater->cloudUpdated[i].connect(boost::bind(&KinectCloudFileWriter<pcl::PointXYZRGB>::pushCloudAsync, m_colorCloudOutputWriter[i], _1));
	}
	m_nonColoredOutputStreamUpdater->cloudsUpdated.connect(boost::bind(&PCLViewer::updateNonColoredClouds, m_pclFaceViewer, _1));
	m_colouredOutputStreamUpdater->cloudsUpdated.connect(boost::bind(&PCLViewer::updateColoredClouds, m_pclFaceViewer, _1));
}

void WindowsApplication::initKinectFrameGrabber()
{
	m_kinectFrameGrabber.setImageRenderer(m_pDrawDataStreams);

	// Get and initialize the default Kinect sensor
	m_kinectFrameGrabber.initializeDefaultSensor();
	int depthWidth, depthHeight, colorWidth, colorHeight;

	m_kinectFrameGrabber.getColourAndDepthSize(depthWidth, depthHeight, colorWidth, colorHeight);

	m_colouredOutputStreamUpdater = std::shared_ptr<ColouredOutputStreamUpdater>(new ColouredOutputStreamUpdater);
	m_nonColoredOutputStreamUpdater = std::shared_ptr<NonColouredOutputStreamsUpdater>(new NonColouredOutputStreamsUpdater);

	m_kinectFrameGrabber.setOutputStreamUpdater(m_colouredOutputStreamUpdater);
	m_kinectFrameGrabber.statusChanged.connect(boost::bind(&WindowsApplication::setStatusMessage, this, _1, _2));

	m_colouredOutputStreamUpdater->initialize(m_kinectFrameGrabber.getCoordinateMapper(), depthWidth, depthHeight, colorWidth, colorHeight);
	m_nonColoredOutputStreamUpdater->initialize(m_kinectFrameGrabber.getCoordinateMapper(), depthWidth, depthHeight, colorWidth, colorHeight);
}

void WindowsApplication::initTabs()
{
	auto recordingConfiguration = initRecordDataModel();
	m_recordTabHandler.setSharedRecordingConfiguration(recordingConfiguration);
	m_recordTabHandle = CreateDialogParamW(
		NULL,
		MAKEINTRESOURCE(IDC_TAB_RECORD),
		m_hWnd,
		(DLGPROC)MessageRouterHelper::MessageRouter,
		reinterpret_cast<LPARAM>(&m_recordTabHandler));
	m_recordTabHandler.colorConfigurationChanged.connect(boost::bind(&WindowsApplication::colorStreamingChangedTo, this, _1));
	m_recordTabHandler.startWriting.connect(boost::bind(&WindowsApplication::startRecording, this, _1, _2));
	m_recordTabHandler.stopWriting.connect(boost::bind(&WindowsApplication::stopRecording, this, _1, _2));

	m_plackBackTabHandler.setSharedRecordingConfiguration(recordingConfiguration);

	m_playbackTabHandle = CreateDialogParamW(
		NULL,
		MAKEINTRESOURCE(IDC_TAB_PLAYBACK),
		m_hWnd,
		(DLGPROC)MessageRouterHelper::MessageRouter,
		reinterpret_cast<LPARAM>(&m_plackBackTabHandler));
	ShowWindow(m_playbackTabHandle, SW_HIDE);
	m_plackBackTabHandler.startPlayback.connect(boost::bind(&WindowsApplication::startPlayback, this, _1, _2));
	m_plackBackTabHandler.stopPlayback.connect(boost::bind(&WindowsApplication::stopPlayback, this));

	m_convertTabHandle = CreateDialogParamW(
		NULL,
		MAKEINTRESOURCE(IDC_TAB_CONVERT),
		m_hWnd,
		(DLGPROC)MessageRouterHelper::MessageRouter,
		reinterpret_cast<LPARAM>(&m_convertTabHandler));
	ShowWindow(m_convertTabHandle, SW_HIDE);

	RECT windowRect;

	GetClientRect(m_hWnd, &windowRect);
	HWND tabControlHandle = GetDlgItem(m_hWnd, IDC_TAB2);

	insertTabItem(tabControlHandle, L"Record", 0);
	insertTabItem(tabControlHandle, L"Playback", 1);
	insertTabItem(tabControlHandle, L"Convert", 2);

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
}


void WindowsApplication::initInputReaderBufferAndSynchronizer()
{
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
}

void WindowsApplication::onTabSelected(int page)
{
	int iPage = TabCtrl_GetCurSel(GetDlgItem(m_hWnd, IDC_TAB2));
	if (iPage == 0){
		if (m_plackBackTabHandler.isPlaybackRunning()){
			TabCtrl_SetCurSel(GetDlgItem(m_hWnd, IDC_TAB2), 1);
		}
		else{
			onRecordTabSelected();
		}

	}
	else if (iPage == 1){
		if (m_recordTabHandler.isRecording()){
			TabCtrl_SetCurSel(GetDlgItem(m_hWnd, IDC_TAB2), 0);
		}
		else{
			onPlaybackSelected();
		}

	}
	else if (iPage == 2){
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
}
void WindowsApplication::connectInputReaderToViewer()
{
	m_bufferSynchronizer.publishSynchronizedData.connect(boost::bind(&PCLViewer::updateColoredClouds, m_pclFaceViewer, _1));
}

void WindowsApplication::disconnectInputReaderFromViewer()
{
	m_bufferSynchronizer.publishSynchronizedData.disconnect_all_slots();
}

void WindowsApplication::onRecordTabSelected()
{
	m_isKinectRunning = true;
	m_plackBackTabHandler.playbackStopped();
	disconnectInputReaderFromViewer();
	connectStreamUpdaterToViewer();
	m_pclFaceViewer->useColoredCloud(m_recordTabHandler.isColorEnabled());
	m_pclFaceViewer->setNumOfClouds(2);
	ShowWindow(m_liveViewWindow, SW_SHOW);
	ShowWindow(m_recordTabHandle, SW_SHOW);
	ShowWindow(m_playbackTabHandle, SW_HIDE);
	ShowWindow(m_convertTabHandle, SW_HIDE);
}

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
	disconnectStreamUpdaterFromViewer();
	connectInputReaderToViewer();
	m_pclFaceViewer->useColoredCloud(true);
	ShowWindow(m_liveViewWindow, SW_HIDE);
	m_plackBackTabHandler.resetUIElements();
	m_plackBackTabHandler.setSharedRecordingConfiguration(m_recordTabHandler.getRecordConfiguration());
	ShowWindow(m_recordTabHandle, SW_HIDE);
	ShowWindow(m_playbackTabHandle, SW_SHOW);
	ShowWindow(m_convertTabHandle, SW_HIDE);
}


void WindowsApplication::startRecording(bool isColoredStream, SharedRecordingConfiguration recordingConfigurations)
{
	if (isColoredStream){
		for (int i = 0; i < RECORD_CLOUD_TYPE_COUNT; i++){
			auto recordingConfig = recordingConfigurations[i];
			auto cloudWriter = m_colorCloudOutputWriter[i];
			if (i == FullDepthRaw){
				m_colouredOutputStreamUpdater->cloudUpdated[i].disconnect_all_slots();
				m_nonColoredOutputStreamUpdater->cloudUpdated[i].disconnect_all_slots();
			}
			
			cloudWriter->setRecordingConfiguration(recordingConfig);
			if (recordingConfig->isEnabled()){
				if (i == FullDepthRaw){
					m_colouredOutputStreamUpdater->cloudUpdated[i].connect(boost::bind(&KinectCloudFileWriter<pcl::PointXYZRGB>::pushCloudAsync, cloudWriter, _1));
				}
				cloudWriter->startWritingClouds();
			}
		}
	}
	else{
		for (int i = 0; i < RECORD_CLOUD_TYPE_COUNT; i++){
			auto recordingConfig = recordingConfigurations[i];
			auto cloudWriter = m_nonColoredCloudOutputWriter[i];
			cloudWriter->setRecordingConfiguration(recordingConfig);
			if (i == FullDepthRaw){
				m_colouredOutputStreamUpdater->cloudUpdated[i].disconnect_all_slots();
				m_nonColoredOutputStreamUpdater->cloudUpdated[i].disconnect_all_slots();
			}
			if (recordingConfig->isEnabled()){
				if (i == FullDepthRaw){
					m_nonColoredOutputStreamUpdater->cloudUpdated[i].connect(boost::bind(&KinectCloudFileWriter<pcl::PointXYZ>::pushCloudAsync, cloudWriter, _1));
				}
				cloudWriter->startWritingClouds();
			}
		}
	}
}


void WindowsApplication::triggerReaderStart(SharedPlaybackConfiguration playbackConfig, bool isSingleThreatedReading)
{
	if (playbackConfig.size() != RECORD_CLOUD_TYPE_COUNT){
		return;
	}

	for (int i = 0; i < RECORD_CLOUD_TYPE_COUNT; i++){
		m_inputFileReader[i]->setPlaybackConfiguration(playbackConfig[i]);
		m_inputFileReader[i]->startReading(isSingleThreatedReading);
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
		inputReader->stopReading();
	}
}

void WindowsApplication::stopRecording(bool isColoredStream, SharedRecordingConfiguration recordingConfigurations)
{
	if (isColoredStream){
		for (int i = 0; i < RECORD_CLOUD_TYPE_COUNT; i++){
			auto recordingConfig = recordingConfigurations[i];
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
			auto recordingConfig = recordingConfigurations[i];
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


