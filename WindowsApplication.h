#pragma once
#include "stdafx.h"
#include "resource.h"
#include "MessageRouterHelper.h"

#include "KinectHDFaceGrabber.h"
#include "resource.h"
#include "ImageRenderer.h"
#include "PCLViewer.h"

#include "KinectCloudFileWriter.h"
#include "KinectRawFileWriter.h"
#include "RecordingConfiguration.h"
#include "ImageRecordingConfiguration.h"
#include "RecordTabHandler.h"
#include "PlaybackTabHandler.h"
#include "ColoredOutputStreamUpdater.h"
#include "UncoloredOutputStreamsUpdater.h"
#include "PCLInputReader.h"
#include "BufferSynchronizer.h"
#include "ConvertTabHandler.h"
#include <thread>
#include "StringFileWriter.h"
#include "StringFileRecordingConfiguration.h"
#include "KinectV1Controller.h"
/**
 * \class	WindowsApplication for the UI, user inputs and management of the model classes
 *
 * \brief	The windows application holds the KinectHDFaceGrabber, FileReader and Writer and processes
 * 			the users input. It constructs the Dataflows neccessary for recording and playing the point cloud
 * 			files.
 */
class WindowsApplication : public MessageRouterHelper
{
public:
	static const int       cColorWidth = 1920;
	static const int       cColorHeight = 1080;

	static const int       cColorWidthForV1 = 640;
	static const int       cColorHeightForV1 = 480;

		
	static const int       cDepthWidthForV1 = 320;
	static const int       cCDepthHeightForV1 = 240;
	WindowsApplication();
	~WindowsApplication();

	/**
	 * \fn	int WindowsApplication::run(HINSTANCE hInstance, int nCmdShow);
	 *
	 * \brief	Runs.
	 *
	 * \param	hInstance	Handle to the application instance
	 * \param	nCmdShow 	whether to display minimized, maximized, or normally
	 *
	 * \return	An int.
	 */

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

	SharedCommonConfiguration initCommonDataModel();

	/**
	* \fn	SharedImageRecordingConfiguration WindowsApplication::initImageRecordDataModel();
	*
	* \brief	Initialises the ImageRecordingConfiguration for the RecordTabHandler class.
	*
	* \return	A SharedImageRecordingConfiguration.
	*/
	SharedImageRecordingConfiguration initImageRecordDataModel();

	SharedStringFileRecordingConfiguration initStringFileRecordDataModel();

	/**
	 * \fn	void WindowsApplication::onPlaybackFinished();
	 *
	 * \brief	Forwards the playback finished signal to the PlaybackTabHandler.
	 */
	void onPlaybackFinished();

	/**
	* \fn	void WindowsApplication::onPlaybackSelected();
	*
	* \brief	Disconnects the stream updater and connects the playback tab handler to the
	* 			PCLViewer instance
	*/
	void onPlaybackSelected();

	/**
	* \fn	void WindowsApplication::onConvertTabSelected()
	*
	* \brief	Hides all other tabs and shows the converstion tab
	*/
	void onConvertTabSelected();

	/**
	* \fn	void WindowsApplication::onRecordTabSelected()
	*
	* \brief	Disconnects the playback handler from the PCLViewer and reconnects the
	* 			record tab handler. Hides all other tabs and shows only the record
	* 			handler
	*/
	void onRecordTabSelected();
		
	/**
	* \fn	void WindowsApplication::UpdateStreams()
	*
	* \brief	update the frame of kinect v1
	* 			
	* 			
	*/

	void UpdateStreams(int i);
	/**
	* \fn	void WindowsApplication::startRecording(bool isColoredStream, SharedRecordingConfiguration recordingConfigurations, SharedImageRecordingConfiguration imageRecordingConfigurations)
	*
	* \brief	Callback to trigger the start of the recording with the specified recording configuration.
	*
	* \param	isColoredStream		   			true if recording should be done with color, or not.
	* \param	recordingConfigurations			The recording configurations.
	* \param	imageRecordingConfigurations	The image recording configurations.
	*/
	void startRecording(bool isColoredStream, SharedRecordingConfiguration recordingConfigurations, 
		SharedImageRecordingConfiguration imageRecordingConfigurations,
		SharedStringFileRecordingConfiguration KeyPointsRecordingConfiguration,
		SharedImageRecordingConfiguration imageRecordingConfigurationsForV1);

	/**
	 * \fn	void WindowsApplication::stopRecording(bool isColoredStream, SharedRecordingConfiguration recordingConfigurations, SharedImageRecordingConfiguration imageRecordingConfigurations);
	 *
	 * \brief	Callback to trigger the stop of the recording with the specified recording configuration.
	 *
	 * \param	isColoredStream					true if this object is colored stream.
	 * \param	recordingConfigurations			The recording configurations.
	 * \param	imageRecordingConfigurations	The image recording configurations.
	 */
	void stopRecording(bool isColoredStream, SharedRecordingConfiguration recordingConfigurations, 
		SharedImageRecordingConfiguration imageRecordingConfigurations,
		SharedStringFileRecordingConfiguration KeyPointsRecordingConfiguration,
		SharedImageRecordingConfiguration imageRecordingConfigurationsForV1);

	/**
	 * \fn	void WindowsApplication::startPlayback(SharedPlaybackConfiguration playbackConfig, bool isSingleThreatedReading);
	 *
	 * \brief	Callback to setup and trigger the start of the playback with the specified playback configuration.
	 *
	 * \param	playbackConfig		   	The playback configuration.
	 * \param	isSingleThreatedReading	true if cloud files should be read single threaded.
	 */
	void startPlayback(SharedPlaybackConfiguration playbackConfig, bool isSingleThreadedReading);


	/**
	 * \fn	void WindowsApplication::triggerReaderStart(SharedPlaybackConfiguration playbackConfig, bool isSingleThreatedReading);
	 *
	 * \brief	Trigger reader to start.
	 *
	 * \param	playbackConfig		   	The playback configuration.
	 * \param	isSingleThreatedReading	true if cloud files should be read single threaded.
	 */

	void triggerReaderStart(SharedPlaybackConfiguration playbackConfig, bool isSingleThreadedReading);

	/**
	 * \fn	void WindowsApplication::setupReaderAndBuffersForPlayback(SharedPlaybackConfiguration playbackConfig);
	 *
	 * \brief	Sets up the reader and buffers for playback.
	 *
	 * \param	playbackConfig	The playback configuration.
	 */
	void setupReaderAndBuffersForPlayback(SharedPlaybackConfiguration playbackConfig);

	/**
	 * \fn	void WindowsApplication::stopPlayback();
	 *
	 * \brief	Stops a playback .
	 */
	void stopPlayback();

	/**
	 * \fn	void WindowsApplication::initCloudWriter();
	 *
	 * \brief	Initialises the cloud writer and registers for status notification.
	 */
	void initCloudWriter();

	/**
	* \fn	void WindowsApplication::initImageWriter();
	*
	* \brief	Initialises the image writer and registers for status notification.
	*/
	void initImageWriter();

	/**
	* \fn	void WindowsApplication::initStringFileWriter();
	*
	* \brief	Initialises the string file writer for keypoints writing.
	*/
	void initStringFileWriter();
	/**
	 * \fn	void WindowsApplication::initInputReaderBufferAndSynchronizer();
	 *
	 * \brief	Initialises the input reader buffer and synchronizer, connects them.
	 * 			The synchronizer thread is started.
	 */
	void initInputReaderBufferAndSynchronizer();

	/**
	 * \fn	void WindowsApplication::initTabs();
	 *
	 * \brief	Creates all tabs, adds their dialog windows redirects the messages and connects
	 * 			the notifications of the tabs to the WindowsApplication.
	 */
	void initTabs();

	/**
	 * \fn	void WindowsApplication::initKinectFrameGrabber();
	 *
	 * \brief	Initialises the kinect frame grabber, image reader and outputstream updater-stragedy and
	 * 			connects them.
	 */
	HRESULT initKinectFrameGrabber();
	
	
	/**
	 * \fn	void WindowsApplication::initKinectV1FrameGrabber();
	 *
	 * \brief	Initialises the kinect v1 frame grabber, image reader and outputstream updater-stragedy and
	 * 			connects them.
	 */
	HRESULT initKinectV1FrameGrabber();

	/**
	 * \fn	void WindowsApplication::connectStreamUpdaterToViewer();
	 *
	 * \brief	Connects the stream updater to viewer and KinectCloudOutputWriter.
	 */
	void connectStreamUpdaterToViewer();

	/**
	* \fn	void WindowsApplication::disconnectStreamUpdaterFromViewer();
	*
	* \brief	Disconnects the stream updater of the kinect grabber from viewer.
	*/
	void disconnectStreamUpdaterFromViewer();

	/**
	 * \fn	void WindowsApplication::connectInputReaderToViewer();
	 *
	 * \brief	Connects the input reader/buffer synchronizer to the PCLViewer.
	 */
	void connectInputReaderToViewer();

	/**
	 * \fn	void WindowsApplication::disconnectInputReaderFromViewer();
	 *
	 * \brief	Disconnects the reader/buffer synchronizer from viewer.
	 */

	void disconnectInputReaderFromViewer();

	/**
	 * \fn	void WindowsApplication::colorStreamingChangedTo(bool enable);
	 *
	 * \brief	Switches the OutputStreamUpdaterStragedy to color or non-colored updating
	 *
	 * \param	enable	true to enable color, false to disable.
	 */
	void colorStreamingChangedTo(bool enable);

	/**
	* \fn	void WindowsApplication::centerRecordingChangedTo(bool enable);
	*
	* \brief	Switches the OutputStreamUpdaterStragedy to centered or non-centered recording
	*
	* \param	enable	true to enable color, false to disable.
	*/
	void centerRecordingChangedTo(bool enable);

	/**
	* \fn	void WindowsApplication::setFPSLimit(int fps,KinectVersionType kinectVersion);
	*
	* \brief	Sets fps limit
	*
	* \param	fps.
	*/
	void setFPSLimit(int fps,KinectVersionType kinectVersion);

	
	/**
	* \fn	void WindowsApplication::setKinectV1AlignmentEnable(bool enable);
	*
	* \brief	enable or disable alignment of KinectV1
	*
	* \param	enable.
	*/
	void setKinectV1AlignmentEnable(bool enable);
	/**
	 * \fn	int WindowsApplication::insertTabItem(HWND tab, LPTSTR text, int tabid);
	 *
	 * \brief	Inserts a tab item into Windows Application tap-control
	 *
	 * \param	tab  	Handle of the tab.
	 * \param	text 	The text.
	 * \param	tabid	The tabid.
	 *
	 * \return	An int.
	 */
	int insertTabItem(HWND tab, LPTSTR text, int tabid);

	/**
	 * \fn	bool WindowsApplication::setStatusMessage(std::wstring statusString, bool bForce);
	 *
	 * \brief	Sets status message in the bottom text field.
	 *
	 * \param	statusString	The status string.
	 * \param	bForce			true to force.
	 *
	 * \return	true if it succeeds, false if it fails.
	 */
	bool setStatusMessage(std::wstring statusString, bool bForce);

	
	/**
	* \fn	SharedImageRecordingConfiguration WindowsApplication::initImageRecordDataModelForKinectV1();
	*
	* \brief	Initialises the ImageRecordingConfiguration of KinectV1 for the RecordTabHandler class.
	*
	* \return	A SharedImageRecordingConfiguration.
	*/
	SharedImageRecordingConfiguration initImageRecordDataModelForKinectV1();
	

	/** \brief	The handle of the windows app instance. */
	HINSTANCE m_hInstance;

	/** \brief	Handle of the record tab. */
	HWND m_recordTabHandle;

	/** \brief	Handle of the playback tab. */
	HWND m_playbackTabHandle;

	/** \brief	Handle of the convert tab. */
	HWND m_convertTabHandle;

	/** \brief	Handle of the live view window. */
	HWND m_liveViewWindow;

	/** \brief	Handle of the live view window. */
	HWND m_liveViewWindow_for_v1;

	/** \brief	The min time delay for the next status update. */
	ULONGLONG m_nNextStatusTime;

	
	/** \brief	The image renderer for the color stream and HDFace. */
	ImageRenderer* m_pDrawDataStreams;

		
	/** \brief	The image renderer for the color stream and HDFace. */
	ImageRenderer* m_pDrawDataStreamsForV1;

	/** \brief	The d 2D factory. */
	ID2D1Factory* m_pD2DFactory;

		/** \brief	The d 2D factory. */
	ID2D1Factory* m_pD2DFactoryForKinectV1;

	/** \brief	The kinect frame grabber. */
	KinectHDFaceGrabber			m_kinectFrameGrabber;

	/** \brief	The PCL face viewer. */
	std::shared_ptr<PCLViewer>	m_pclFaceViewer;

	/** \brief	The input file reader. */
	std::vector<std::shared_ptr<PCLInputReader<pcl::PointXYZRGB>>>	m_inputFileReader;
	
	/** \brief	The color cloud writer. */
	std::vector<std::shared_ptr<KinectCloudFileWriter<pcl::PointXYZRGB>>> m_colorCloudOutputWriter;

	/** \brief	The non colored cloud writer. */
	std::vector<std::shared_ptr<KinectCloudFileWriter<pcl::PointXYZ>>> m_uncoloredCloudOutputWriter;

	/** \brief	The image writer for kinect v2. */
	std::vector<std::shared_ptr<KinectRawFileWriter>> m_imageOutputWriter;

	/** \brief	The image writer for kinect v1. */
	std::vector<std::shared_ptr<KinectRawFileWriter>> m_kinectV1ImageOutputWriter;

	std::vector<std::shared_ptr<StringFileWriter>> m_stringFileOutputWriter;
	/** \brief	The image writer. */
	//std::vector<std::shared_ptr<KinectRawFileWriter>> m_imageOutputWriter;

	/** \brief	The buffer synchronizer for reading files. */
	BufferSynchronizer<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> m_bufferSynchronizer;

	/** \brief	The buffer synchronizer thread. */
	std::thread	m_bufferSynchronizerThread;

	/** \brief	true if kinect updating should be triggered. */
	bool m_isKinectRunning;

	/** \brief	The record tab handler. */
	RecordTabHandler m_recordTabHandler;

	/** \brief	The plack back tab handler. */
	PlaybackTabHandler m_plackBackTabHandler;

	/** \brief	The convert tab handler. */
	ConvertTabHandler m_convertTabHandler;

	/** \brief	The coloured output stream updater. */
	std::shared_ptr<ColoredOutputStreamUpdater> m_coloredOutputStreamUpdater;

	/** \brief	The non colored output stream updater. */
	std::shared_ptr<UncoloredOutputStreamsUpdater> m_uncoloredOutputStreamUpdater;

	std::shared_ptr<KinectV1OutPutStreamUpdater> m_kinectV1OutputStreamUpdater;
	
	/** \brief indicate if kinect v1 is running */
	bool m_kinectV1Enable;

	/** \brief indicate if kinect v2 is running */
	bool m_kinectV2Enable;

	/** \brief	FPS Limit. */
	int m_FPSLimit;

	KinectV1Controller m_kinectV1Controller;
	//UINT m_v1SleepMillsecond;
public:
	/* \brief exit the thread of V1*/
	HANDLE m_hStopStreamEventThread;

	HANDLE m_hPauseStreamEventThread;

	/* \brief get the Handle of the current window*/
	HWND GetWindow() const;

	/** \brief	thread for update stream of V1. */
	static DWORD WINAPI runKinectV1StreamEvent(WindowsApplication * pThis);
	static DWORD WINAPI runKinectV2Update(WindowsApplication * pThis);
	
	/* \brief set the title angle for kinect v1*/
	void  setTitleAngle(LONG degree);

	/** \mutex for protecting the opereation of kinect v1 in two threads */
	std::mutex m_kinectV1DataUpdateMutex;
	
};

