#pragma once
#include "stdafx.h"
#include "resource.h"
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
#include <thread>
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

	

	void cloudUpdate(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud);
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
	
	static bool openFileDialog(WCHAR* szDir, HWND handle);
	static bool openDirectoryDialog(WCHAR* szDir, HWND handle);

private:
	void onCreate();
	void onSelectionChanged(WPARAM wParam, LPARAM handle);
	void onButtonClicked(WPARAM wParam, LPARAM handle);
	void onEditBoxeChanged(WPARAM wParam, LPARAM handle);
	void recordPathChanged(RecordCloudType type);
	void recordConfigurationStatusChanged(RecordCloudType type, bool newState);

	void initRecordDataModel();
	void checkRecordingConfigurationPossible();
	void onPlaybackFinished();

	void onPlaybackSelected();
	void onRecordTabSelected();
	void startRecording(bool isColoredStream);
	void stopRecording(bool isColoredStream);

	void startPlayback(SharedPlaybackConfiguration playbackConfig);
	void stopPlayback();

	void processUIMessage(WPARAM wParam, LPARAM);
	void connectWriterAndViewerToKinect();
	void connectInputReaderToViewer();

	void disconnectWriterAndViewerToKinect();
	void disconnectInputReaderFromViewer();
	void colorStreamingChangedTo(bool enable);

	bool					setStatusMessage(std::wstring statusString, bool bForce);
	HINSTANCE				m_hInstance;
	HWND					m_hWnd;
	HWND					m_recordTabHandle;
	HWND					m_playbackTabHandle;

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
	std::vector<std::shared_ptr<PCLInputReader>>							m_inputFileReader;

	//pcl::visualization::CloudViewer m_cloudViewer;

	std::vector<std::shared_ptr<KinectCloudOutputWriter<pcl::PointXYZRGB>>> m_colorCloudOutputWriter;
	std::vector<std::shared_ptr<KinectCloudOutputWriter<pcl::PointXYZ>>>	m_nonColoredCloudOutputWriter;
	
	//RecordingConfiguration	m_recordingConfiguration[RECORD_CLOUD_TYPE_COUNT];
	SharedRecordingConfiguration	m_recordingConfiguration;
	//std::shared_ptr<RecordingConfiguration>	m_recordingConfiguration;
	BufferSynchronizer<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>				m_bufferSynchronizer;
	std::thread																m_bufferSynchronizerThread;
	bool																	m_isKinectRunning;
	RecordTabHandler														m_recordTabHandler;
	PlaybackTabHandler														m_plackBackTabHandler;
	std::shared_ptr<ColouredOutputStreamUpdater>							m_colouredOutputStreamUpdater;
	std::shared_ptr<NonColouredOutputStreamsUpdater>						m_nonColoredOutputStreamUpdater;
};

