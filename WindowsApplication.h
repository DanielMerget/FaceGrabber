#pragma once
#include "stdafx.h"
#include "resource.h"
#include "MessageRouterHelper.h"

#include "KinectHDFaceGrabber.h"
#include "resource.h"
#include "ImageRenderer.h"
#include "PCLViewer.h"
#include <memory>
#include "KinectCloudOutputWriter.h"
#include <pcl/visualization/image_viewer.h>
#include <pcl/visualization/cloud_viewer.h>
#include "RecordingConfiguration.h"
#include "RecordTabHandler.h"
#include "PlaybackTabHandler.h"
#include "ColouredOutputStreamUpdater.h"
#include "NonColouredOutputStreamsUpdater.h"
#include "PCLInputReader.h"
#include "BufferSynchronizer.h"
#include "ConvertTabHandler.h"
#include <thread>

class WindowsApplication : public MessageRouterHelper
{
public:


	static const int       cColorWidth = 1920;
	static const int       cColorHeight = 1080;


	WindowsApplication();
	~WindowsApplication();

	void imageUpdated(const unsigned char *data, unsigned width, unsigned height);

	

	void cloudUpdate(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud);

	
	int						run(HINSTANCE hInstance, int nCmdShow);

private:
	void onCreate();
	
	void onTabSelected(int page);
	
	void initRecordDataModel();

	void onPlaybackFinished();
	void onPlaybackSelected();
	void onConvertTabSelected();
	void onRecordTabSelected();
	void startRecording(bool isColoredStream);
	void stopRecording(bool isColoredStream);

	void startPlayback(SharedPlaybackConfiguration playbackConfig, bool isSingleThreatedReading);
	void triggerReaderStart(SharedPlaybackConfiguration playbackConfig, bool isSingleThreatedReading);
	void setupReaderAndBuffersForPlayback(SharedPlaybackConfiguration playbackConfig);
	void stopPlayback();

	void connectWriterAndViewerToKinect();
	void connectInputReaderToViewer();

	void disconnectWriterAndViewerToKinect();
	void disconnectInputReaderFromViewer();
	void colorStreamingChangedTo(bool enable);

	bool					setStatusMessage(std::wstring statusString, bool bForce);
	HINSTANCE				m_hInstance;
	//HWND					m_hWnd;
	HWND					m_recordTabHandle;
	HWND					m_playbackTabHandle;
	HWND					m_convertTabHandle;


	HWND					m_liveViewWindow;
	INT64					m_nStartTime;
	INT64					m_nLastCounter;
	double					m_fFreq;
	ULONGLONG				m_nNextStatusTime;
	DWORD					m_nFramesSinceUpdate;

	// Direct2D
	ImageRenderer*         m_pDrawDataStreams;
	ID2D1Factory*          m_pD2DFactory;
	bool				   m_isCloudWritingStarted;

	KinectHDFaceGrabber			m_kinectFrameGrabber;
	std::shared_ptr<PCLViewer>	m_pclFaceViewer;
	std::vector<std::shared_ptr<PCLInputReader<pcl::PointXYZRGB>>>			m_inputFileReader;
	
	std::vector<std::shared_ptr<KinectCloudOutputWriter<pcl::PointXYZRGB>>> m_colorCloudOutputWriter;
	std::vector<std::shared_ptr<KinectCloudOutputWriter<pcl::PointXYZ>>>	m_nonColoredCloudOutputWriter;
	
	SharedRecordingConfiguration	m_recordingConfiguration;
	BufferSynchronizer<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>				m_bufferSynchronizer;
	std::thread																m_bufferSynchronizerThread;
	bool																	m_isKinectRunning;
	RecordTabHandler														m_recordTabHandler;
	PlaybackTabHandler														m_plackBackTabHandler;
	ConvertTabHandler														m_convertTabHandler;
	std::shared_ptr<ColouredOutputStreamUpdater>							m_colouredOutputStreamUpdater;
	std::shared_ptr<NonColouredOutputStreamsUpdater>						m_nonColoredOutputStreamUpdater;	
};

