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

	// Create and initialize a new Direct2D image renderer 
	m_pDrawDataStreams = new ImageRenderer();

	//Create the pcl viewer
	m_pclFaceViewer = std::shared_ptr<PCLViewer>(new PCLViewer(2, "Face-Viewer"));

	
	initKinectFrameGrabber();
	initTabs();
	initCloudWriter();
	initImageWriter();
	connectStreamUpdaterToViewer();

	initInputReaderBufferAndSynchronizer();

	//set the settings of the views and frames 
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
	//disconnect all slots, so e.g. the PCLViewer does not get updated any more
	for (int i = 0; i < RECORD_CLOUD_TYPE_COUNT; i++){
		//No Signals connected for both Raw Images
		if (i == KinectColorRaw || i == KinectDepthRaw){
			continue;
		}
		m_uncoloredOutputStreamUpdater->cloudUpdated[i].disconnect_all_slots();
		m_coloredOutputStreamUpdater->cloudUpdated[i].disconnect_all_slots();
	}
	m_uncoloredOutputStreamUpdater->cloudsUpdated.disconnect_all_slots();
	m_coloredOutputStreamUpdater->cloudsUpdated.disconnect_all_slots();
}


void WindowsApplication::initCloudWriter()
{
	for (int i = 0; i < RECORD_CLOUD_TYPE_COUNT; i++){
		//No CloudFileWriter for both Raw Images
		if (i == KinectColorRaw || i == KinectDepthRaw){
			continue;
		}
		//init
		auto uncoloredCloudWriter = std::shared_ptr<KinectCloudFileWriter<pcl::PointXYZ>>(new KinectCloudFileWriter<pcl::PointXYZ>);
		//register for events
		uncoloredCloudWriter->updateStatus.connect(boost::bind(&RecordTabHandler::updateWriterStatus, &m_recordTabHandler, static_cast<RecordCloudType>(i), _1));
		uncoloredCloudWriter->writingWasStopped.connect(boost::bind(&RecordTabHandler::recordingStopped, &m_recordTabHandler));

		m_uncoloredCloudOutputWriter.push_back(uncoloredCloudWriter);

		//init
		auto coloredCloudWriter = std::shared_ptr<KinectCloudFileWriter<pcl::PointXYZRGB>>(new KinectCloudFileWriter<pcl::PointXYZRGB>);
		//register for events
		coloredCloudWriter->updateStatus.connect(boost::bind(&RecordTabHandler::updateWriterStatus, &m_recordTabHandler, static_cast<RecordCloudType>(i), _1));
		coloredCloudWriter->writingWasStopped.connect(boost::bind(&RecordTabHandler::recordingStopped, &m_recordTabHandler));

		m_colorCloudOutputWriter.push_back(coloredCloudWriter);
	}
}

void WindowsApplication::initImageWriter()
{
	for (int i = 0; i < RECORD_CLOUD_TYPE_COUNT; i++){
		if (i == KinectColorRaw){
			//init
			auto imageWriter = std::shared_ptr<KinectRawFileWriter>(new KinectRawFileWriter);
			//register for events
			imageWriter->updateStatus.connect(boost::bind(&RecordTabHandler::updateWriterStatus, &m_recordTabHandler, static_cast<RecordCloudType>(i), _1));
			imageWriter->writingWasStopped.connect(boost::bind(&RecordTabHandler::recordingStopped, &m_recordTabHandler));

			m_colorImageOutputWriter.push_back(imageWriter);
		}
		if (i == KinectDepthRaw)
		{
			//init
			auto imageWriter = std::shared_ptr<KinectRawFileWriter>(new KinectRawFileWriter);
			//register for events
			imageWriter->updateStatus.connect(boost::bind(&RecordTabHandler::updateWriterStatus, &m_recordTabHandler, static_cast<RecordCloudType>(i), _1));
			imageWriter->writingWasStopped.connect(boost::bind(&RecordTabHandler::recordingStopped, &m_recordTabHandler));

			m_depthImageOutputWriter.push_back(imageWriter);
		}
	}
}

void WindowsApplication::connectStreamUpdaterToViewer()
{
	for (int i = 0; i < RECORD_CLOUD_TYPE_COUNT; i++){
		//we skip enabling the fulldepth raw and kinect raw; this signal is connected 
		//only if recording is enabled; this way, full depth buffer conversion is skipped 
		//if not recorded
		if (i == FullDepthRaw || i == KinectColorRaw || i == KinectDepthRaw){
			continue;
		}
		//bind to the writer
		m_uncoloredOutputStreamUpdater->cloudUpdated[i].connect(boost::bind(&KinectCloudFileWriter<pcl::PointXYZ>::pushCloudAsync, m_uncoloredCloudOutputWriter[i], _1));
		m_coloredOutputStreamUpdater->cloudUpdated[i].connect(boost::bind(&KinectCloudFileWriter<pcl::PointXYZRGB>::pushCloudAsync, m_colorCloudOutputWriter[i], _1));
	}
	//bind to the viewer
	m_uncoloredOutputStreamUpdater->cloudsUpdated.connect(boost::bind(&PCLViewer::updateUncoloredClouds, m_pclFaceViewer, _1));
	m_coloredOutputStreamUpdater->cloudsUpdated.connect(boost::bind(&PCLViewer::updateColoredClouds, m_pclFaceViewer, _1));
}

void WindowsApplication::initKinectFrameGrabber()
{
	m_kinectFrameGrabber.setImageRenderer(m_pDrawDataStreams);

	// Get and initialize the default Kinect sensor
	m_kinectFrameGrabber.initializeDefaultSensor();
	int depthWidth, depthHeight, colorWidth, colorHeight;

	m_kinectFrameGrabber.getColourAndDepthSize(depthWidth, depthHeight, colorWidth, colorHeight);

	//init stragedy
	m_coloredOutputStreamUpdater = std::shared_ptr<ColoredOutputStreamUpdater>(new ColoredOutputStreamUpdater);
	m_uncoloredOutputStreamUpdater = std::shared_ptr<UncoloredOutputStreamsUpdater>(new UncoloredOutputStreamsUpdater);

	//set the colored as default
	m_kinectFrameGrabber.setOutputStreamUpdater(m_coloredOutputStreamUpdater);

	//register for status messages (HDFace grabbing state)
	m_kinectFrameGrabber.statusChanged.connect(boost::bind(&WindowsApplication::setStatusMessage, this, _1, _2));

	//init the stragedy
	m_coloredOutputStreamUpdater->initialize(m_kinectFrameGrabber.getCoordinateMapper(), depthWidth, depthHeight, colorWidth, colorHeight);
	m_uncoloredOutputStreamUpdater->initialize(m_kinectFrameGrabber.getCoordinateMapper(), depthWidth, depthHeight, colorWidth, colorHeight);
}

void WindowsApplication::initTabs()
{
	auto recordingConfiguration = initRecordDataModel();
	m_recordTabHandler.setSharedRecordingConfiguration(recordingConfiguration);

	//record, playback and convert are "subdialogs" with own message routers. They process their UI messages on their own
	//we will show only one of those subdialogs once
	//init record tab
	m_recordTabHandle = CreateDialogParamW(
		NULL,
		MAKEINTRESOURCE(IDC_TAB_RECORD),
		m_hWnd,
		(DLGPROC)MessageRouterHelper::MessageRouter,
		reinterpret_cast<LPARAM>(&m_recordTabHandler));
	m_recordTabHandler.colorConfigurationChanged.connect(boost::bind(&WindowsApplication::colorStreamingChangedTo, this, _1));
	m_recordTabHandler.centerConfigurationChanged.connect(boost::bind(&WindowsApplication::centerRecordingChangedTo, this, _1));
	m_recordTabHandler.startWriting.connect(boost::bind(&WindowsApplication::startRecording, this, _1, _2));
	m_recordTabHandler.stopWriting.connect(boost::bind(&WindowsApplication::stopRecording, this, _1, _2));

	m_plackBackTabHandler.setSharedRecordingConfiguration(recordingConfiguration);

	//init playbackt tab
	m_playbackTabHandle = CreateDialogParamW(
		NULL,
		MAKEINTRESOURCE(IDC_TAB_PLAYBACK),
		m_hWnd,
		(DLGPROC)MessageRouterHelper::MessageRouter,
		reinterpret_cast<LPARAM>(&m_plackBackTabHandler));
	ShowWindow(m_playbackTabHandle, SW_HIDE);
	m_plackBackTabHandler.startPlayback.connect(boost::bind(&WindowsApplication::startPlayback, this, _1, _2));
	m_plackBackTabHandler.stopPlayback.connect(boost::bind(&WindowsApplication::stopPlayback, this));

	//init conver tab
	m_convertTabHandle = CreateDialogParamW(
		NULL,
		MAKEINTRESOURCE(IDC_TAB_CONVERT),
		m_hWnd,
		(DLGPROC)MessageRouterHelper::MessageRouter,
		reinterpret_cast<LPARAM>(&m_convertTabHandler));
	ShowWindow(m_convertTabHandle, SW_HIDE);

	

	//create the tab items
	HWND tabControlHandle = GetDlgItem(m_hWnd, IDC_TAB2);	
	insertTabItem(tabControlHandle, L"Record", 0);
	insertTabItem(tabControlHandle, L"Playback", 1);
	insertTabItem(tabControlHandle, L"Convert", 2);
	//set the default
	TabCtrl_SetCurSel(tabControlHandle, 0);
	ShowWindow(tabControlHandle, SW_SHOW);

	//get the areas to adjust the size of the color live stream dialog/view
	RECT windowRect;
	GetClientRect(m_hWnd, &windowRect);

	RECT tabControlRect;
	GetWindowRect(tabControlHandle, &tabControlRect);

	//create subwindow for the color live stream
	const int width = 840;
	const int height = (width / 16) * 9;
	const int xPos = (windowRect.right - windowRect.left - width) / 2;
	m_liveViewWindow = CreateWindow(WC_STATIC, L"", WS_CHILD | WS_VISIBLE, xPos, tabControlRect.top, width, height, m_hWnd, NULL, m_hInstance, NULL);
}


void WindowsApplication::initInputReaderBufferAndSynchronizer()
{
	//create the buffers and store them for the synchronizer
	std::vector<std::shared_ptr<Buffer<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>>> buffers;
	for (int i = 0; i < RECORD_CLOUD_TYPE_COUNT; i++){
		//No Playback for both Raw Images
		if (i == KinectColorRaw || i == KinectDepthRaw){
			continue;
		}
		//create buffer & input reader
		auto buffer = std::shared_ptr<Buffer<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>>(new Buffer<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>);
		auto inputReader = std::shared_ptr<PCLInputReader<pcl::PointXYZRGB>>(new PCLInputReader<pcl::PointXYZRGB>());

		//forward the updates of the reader to the playbacktabhandler
		inputReader->updateStatus.connect(boost::bind(&PlaybackTabHandler::updateReaderStatus, &m_plackBackTabHandler, static_cast<RecordCloudType>(i), _1));
		inputReader->setBuffer(buffer);

		//forward the status event of the buffersynchronizer to the playbacktabhandler
		m_bufferSynchronizer.updateStatus.connect(boost::bind(&PlaybackTabHandler::updateReaderStatus, &m_plackBackTabHandler, static_cast<RecordCloudType>(i), _1));

		//store reader & writer
		m_inputFileReader.push_back(inputReader);
		buffers.push_back(buffer);
	}

	//register for the playback finished event
	m_bufferSynchronizer.playbackFinished.connect(boost::bind(&WindowsApplication::onPlaybackFinished, this));

	//start the buffer synchronizer thread => will sleep until buffer notifes
	m_bufferSynchronizerThread = std::thread(&BufferSynchronizer<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>::updateThreadFunc, &m_bufferSynchronizer);
}

void WindowsApplication::onTabSelected(int page)
{
	//only display one of the tabs at once
	//avoid clicks by the user while playback/recording is running
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

	//hide the other views
	ShowWindow(m_playbackTabHandle, SW_HIDE);
	ShowWindow(m_convertTabHandle, SW_HIDE);
}

void WindowsApplication::onConvertTabSelected()
{
	//hide the other views
	ShowWindow(m_liveViewWindow, SW_HIDE);
	ShowWindow(m_recordTabHandle, SW_HIDE);
	ShowWindow(m_playbackTabHandle, SW_HIDE);

	ShowWindow(m_convertTabHandle, SW_SHOW);
}

void WindowsApplication::onPlaybackSelected()
{
	setStatusMessage(L"", true);
	//stop the updating of the kinect frame grabber
	m_isKinectRunning = false;

	disconnectStreamUpdaterFromViewer();
	connectInputReaderToViewer();

	//preset to default
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
	//use the correct recoder & updater
	if (isColoredStream){
		for (int i = 0; i < RECORD_CLOUD_TYPE_COUNT; i++){
			auto recordingConfig = recordingConfigurations[i];
			if (i == KinectColorRaw){
				auto writer = m_colorImageOutputWriter[0];

				//make sure the KinectColorRaw was disconnected from last recording session
				//we only want to do that image creation if required
				m_coloredOutputStreamUpdater->colorImageUpdated.disconnect_all_slots();
				m_uncoloredOutputStreamUpdater->colorImageUpdated.disconnect_all_slots();
				if (recordingConfig->isEnabled()){
					writer->setRecordingConfiguration(recordingConfig);
					m_coloredOutputStreamUpdater->colorImageUpdated.connect(boost::bind(&KinectRawFileWriter::pushImageAsync, writer, _1));
					writer->startWriting();
				}
			}
			else if (i == KinectDepthRaw){
				auto writer = m_depthImageOutputWriter[0];

				//make sure the KinectDepthRaw was disconnected from last recording session
				//we only want to do that image creation if required
				m_coloredOutputStreamUpdater->depthImageUpdated.disconnect_all_slots();
				m_uncoloredOutputStreamUpdater->depthImageUpdated.disconnect_all_slots();
				if (recordingConfig->isEnabled()){
					writer->setRecordingConfiguration(recordingConfig);
					m_coloredOutputStreamUpdater->depthImageUpdated.connect(boost::bind(&KinectRawFileWriter::pushImageAsync, writer, _1));
					writer->startWriting();
				}
			}
			else{
				auto writer = m_colorCloudOutputWriter[i];
				if (i == FullDepthRaw){
					//make sure the FullDepthRaw was disconnected from last recording session
					//we only want to do that point cloud creation if required
					m_coloredOutputStreamUpdater->cloudUpdated[i].disconnect_all_slots();
					m_uncoloredOutputStreamUpdater->cloudUpdated[i].disconnect_all_slots();
				}
			
				writer->setRecordingConfiguration(recordingConfig);
				if (recordingConfig->isEnabled()){
					//we agreed on manually enabling the FullDepthRaw file writer, so we do not that cloud if not required
					if (i == FullDepthRaw){
						m_coloredOutputStreamUpdater->cloudUpdated[i].connect(boost::bind(&KinectCloudFileWriter<pcl::PointXYZRGB>::pushCloudAsync, writer, _1));
					}
					writer->startWriting();
				}
			}
		}
	}
	else{
		for (int i = 0; i < RECORD_CLOUD_TYPE_COUNT; i++){
			auto recordingConfig = recordingConfigurations[i];
			if (i == KinectColorRaw){
				auto writer = m_colorImageOutputWriter[0];

				//make sure the KinectColorRaw was disconnected from last recording session
				//we only want to do that image creation if required
				m_coloredOutputStreamUpdater->colorImageUpdated.disconnect_all_slots();
				m_uncoloredOutputStreamUpdater->colorImageUpdated.disconnect_all_slots();
				if (recordingConfig->isEnabled()){
					writer->setRecordingConfiguration(recordingConfig);
					m_uncoloredOutputStreamUpdater->colorImageUpdated.connect(boost::bind(&KinectRawFileWriter::pushImageAsync, writer, _1));
					writer->startWriting();
				}
			}
			else if (i == KinectDepthRaw){
				auto writer = m_depthImageOutputWriter[0];

				//make sure the KinectDepthRaw was disconnected from last recording session
				//we only want to do that image creation if required
				m_coloredOutputStreamUpdater->depthImageUpdated.disconnect_all_slots();
				m_uncoloredOutputStreamUpdater->depthImageUpdated.disconnect_all_slots();
				if (recordingConfig->isEnabled()){
					writer->setRecordingConfiguration(recordingConfig);
					m_uncoloredOutputStreamUpdater->depthImageUpdated.connect(boost::bind(&KinectRawFileWriter::pushImageAsync, writer, _1));
					writer->startWriting();
				}
			}
			else{
				auto writer = m_uncoloredCloudOutputWriter[i];
				if (i == FullDepthRaw){
					//make sure the FullDepthRaw was disconnected from last recording session
					//we only want to do that point cloud creation if required
					m_coloredOutputStreamUpdater->cloudUpdated[i].disconnect_all_slots();
					m_uncoloredOutputStreamUpdater->cloudUpdated[i].disconnect_all_slots();
				}

				if (recordingConfig->isEnabled()){
					if (i == FullDepthRaw){
						//we agreed on manually enabling the FullDepthRaw file writer, so we do not that cloud if not required
						m_uncoloredOutputStreamUpdater->cloudUpdated[i].connect(boost::bind(&KinectCloudFileWriter<pcl::PointXYZ>::pushCloudAsync, writer, _1));
					}
					writer->startWriting();
				}
			}
		}
	}
}


void WindowsApplication::triggerReaderStart(SharedPlaybackConfiguration playbackConfig, bool isSingleThreatedReading)
{
	if (playbackConfig.size() != RECORD_CLOUD_TYPE_COUNT){
		return;
	}
	//start all readers; reading for playbackConfig is disable the inputreader will finish immediately
	for (int i = 0; i < RECORD_CLOUD_TYPE_COUNT; i++){
		//No Playback for both Raw Images
		if (i == KinectColorRaw || i == KinectDepthRaw){
			continue;
		}
		m_inputFileReader[i]->setPlaybackConfiguration(playbackConfig[i]);
		m_inputFileReader[i]->startReading(isSingleThreatedReading);
	}
}
void WindowsApplication::setupReaderAndBuffersForPlayback(SharedPlaybackConfiguration playbackConfig)
{
	int enabledClouds = 0;
	std::vector<std::shared_ptr<Buffer<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>>> activeBuffers;

	//we have to specify the number of files which will be read, so the buffersynchronizer can notify the tab
	//when all clouds were rendered
	//we need to find the buffers are used for that playback, so the buffersynchronizer waits only for the enabled
	//ones
	int numOfFilesToRead = 0;
	for (int i = 0; i < RECORD_CLOUD_TYPE_COUNT; i++){
		//No Playback for both Raw Images
		if (i == KinectColorRaw || i == KinectDepthRaw){
			continue;
		}
		auto& currentConfig = playbackConfig[i];
		if (currentConfig->isEnabled()){
			enabledClouds++;
			numOfFilesToRead = currentConfig->getCloudFilesToPlay().size();
			activeBuffers.push_back(m_inputFileReader[i]->getBuffer());
		}
	}

	//set the buffers and adjust the number of clouds to show in the viewer
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
	//stop the correct writer
	if (isColoredStream){
		for (int i = 0; i < RECORD_CLOUD_TYPE_COUNT; i++){
			auto recordingConfig = recordingConfigurations[i];
			auto cloudWriter = m_colorCloudOutputWriter[i];

			//stop those which were enabled/started
			if (recordingConfig->isEnabled()){
				cloudWriter->stopWriting();
			}

			//disconect again so FullDepthRaw is not created anymore
			if (i == FullDepthRaw){
				m_coloredOutputStreamUpdater->cloudUpdated[i].disconnect_all_slots();
			}
		}
	}
	else{
		for (int i = 0; i < RECORD_CLOUD_TYPE_COUNT; i++){
			auto recordingConfig = recordingConfigurations[i];
			auto cloudWriter = m_uncoloredCloudOutputWriter[i];

			//stop those which were enabled/started
			if (recordingConfig->isEnabled()){
				cloudWriter->stopWriting();
			}
			//disconect again so FullDepthRaw is not created anymore
			if (i == FullDepthRaw){
				m_uncoloredOutputStreamUpdater->cloudUpdated[i].disconnect_all_slots();
			}
		}
	}
}


void WindowsApplication::colorStreamingChangedTo(bool enable)
{
	//chose the correct implementation for the stragedy
	if (enable){
		m_kinectFrameGrabber.setOutputStreamUpdater(m_coloredOutputStreamUpdater);
	}
	else{
		m_kinectFrameGrabber.setOutputStreamUpdater(m_uncoloredOutputStreamUpdater);
	}
	//let the PCLViwer know which of the clouds will be updated
	m_pclFaceViewer->useColoredCloud(enable);
}

void WindowsApplication::centerRecordingChangedTo(bool enable)
{
	m_coloredOutputStreamUpdater->setCeterEnabled(enable);
	m_uncoloredOutputStreamUpdater->setCeterEnabled(enable);
}

bool WindowsApplication::setStatusMessage(std::wstring statusString, bool bForce)
{
	//only update if neccessary
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


