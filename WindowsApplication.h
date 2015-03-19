#pragma once
#include "stdafx.h"
#include "resource.h"
#include "MessageRouterHelper.h"

#include "KinectHDFaceGrabber.h"
#include "resource.h"
#include "ImageRenderer.h"
#include "PCLViewer.h"

#include "KinectCloudOutputWriter.h"
#include "RecordingConfiguration.h"
#include "RecordTabHandler.h"
#include "PlaybackTabHandler.h"
#include "ColouredOutputStreamUpdater.h"
#include "NonColouredOutputStreamsUpdater.h"
#include "PCLInputReader.h"
#include "BufferSynchronizer.h"
#include "ConvertTabHandler.h"
#include <thread>

/**
 * \class	WindowsApplication for the UI, user inputs and management of the model classes
 *
 * \brief	The windows application holds the KinectHDFaceGrabber, FileReader and Writer and processes
 * 			the users input.
 */

class WindowsApplication : public MessageRouterHelper
{
public:
	static const int       cColorWidth = 1920;
	static const int       cColorHeight = 1080;

	WindowsApplication();
	~WindowsApplication();

	int	run(HINSTANCE hInstance, int nCmdShow);

private:

	/**
	 * \fn	void WindowsApplication::onCreate();
	 *
	 * \brief	Creates the entire UI, model objects and connects them via signals.
	 */

	void onCreate();

	/**
	 * \fn	void WindowsApplication::onTabSelected(int page);
	 *
	 * \brief	shows and/or hides the the tabs and reconnects the signals.
	 *
	 * \param	page	The page.
	 */

	void onTabSelected(int page);

	/**
	 * \fn	SharedRecordingConfiguration WindowsApplication::initRecordDataModel();
	 *
	 * \brief	Initialises the RecordingConfiguration for the RecordTabHandler class.
	 *
	 * \return	A SharedRecordingConfiguration.
	 */

	SharedRecordingConfiguration initRecordDataModel();

	
	void onPlaybackFinished();
	void onPlaybackSelected();
	void onConvertTabSelected();
	void onRecordTabSelected();
	void startRecording(bool isColoredStream, SharedRecordingConfiguration recordingConfigurations);
	void stopRecording(bool isColoredStream, SharedRecordingConfiguration recordingConfigurations);

	void startPlayback(SharedPlaybackConfiguration playbackConfig, bool isSingleThreatedReading);
	void triggerReaderStart(SharedPlaybackConfiguration playbackConfig, bool isSingleThreatedReading);
	void setupReaderAndBuffersForPlayback(SharedPlaybackConfiguration playbackConfig);
	void stopPlayback();

	void initCloudWriter();
	void initInputReaderBufferAndSynchronizer();
	void initTabs();
	void initKinectFrameGrabber();

	void connectStreamUpdaterToViewer();
	void connectInputReaderToViewer();
	void disconnectStreamUpdaterFromViewer();
	void disconnectInputReaderFromViewer();
	void colorStreamingChangedTo(bool enable);

	int insertTabItem(HWND tab, LPTSTR text, int tabid);

	bool setStatusMessage(std::wstring statusString, bool bForce);

	HINSTANCE m_hInstance;
	HWND m_recordTabHandle;
	HWND m_playbackTabHandle;
	HWND m_convertTabHandle;
	HWND m_liveViewWindow;

	ULONGLONG m_nNextStatusTime;

	// Direct2D
	ImageRenderer* m_pDrawDataStreams;
	ID2D1Factory* m_pD2DFactory;
	bool m_isCloudWritingStarted;

	KinectHDFaceGrabber			m_kinectFrameGrabber;
	std::shared_ptr<PCLViewer>	m_pclFaceViewer;
	std::vector<std::shared_ptr<PCLInputReader<pcl::PointXYZRGB>>>	m_inputFileReader;
	
	std::vector<std::shared_ptr<KinectCloudOutputWriter<pcl::PointXYZRGB>>> m_colorCloudOutputWriter;
	std::vector<std::shared_ptr<KinectCloudOutputWriter<pcl::PointXYZ>>> m_nonColoredCloudOutputWriter;
	
	//SharedRecordingConfiguration	m_recordingConfiguration;
	BufferSynchronizer<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> m_bufferSynchronizer;
	std::thread	m_bufferSynchronizerThread;
	bool m_isKinectRunning;
	RecordTabHandler m_recordTabHandler;
	PlaybackTabHandler m_plackBackTabHandler;
	ConvertTabHandler m_convertTabHandler;
	std::shared_ptr<ColouredOutputStreamUpdater> m_colouredOutputStreamUpdater;
	std::shared_ptr<NonColouredOutputStreamsUpdater> m_nonColoredOutputStreamUpdater;	
};

