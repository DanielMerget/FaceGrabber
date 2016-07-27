#pragma once
#include "stdafx.h"

#include "MessageRouterHelper.h"
#include <windowsx.h>
#include "resource.h"
#include "RecordingConfiguration.h"
#include "ImageRecordingConfiguration.h"
#include <memory>
#include <boost/signals.hpp>
#include "StringFileRecordingConfiguration.h"
#include "CommonConfiguration.h"
#include "KinectV1Controller.h"

enum kinectEnabledOpt{
	OnlyKinectV2Enabled,
	OnlyKinectV1Enabled,
	BothKinectEnabled,
	NoneEnable
};
/**
 * \class	RecordTabHandler
 *
 * \brief	RecordTabHandle handles all user inputs and configures the RecordingConfiguration used to record
 * 			point cloud files. If requested by the user, the RecordTabHandler fires the startWriting and stopWriting
 * 			signals providing the RecordingConfiguration.
 */

class RecordTabHandler : public MessageRouterHelper
{
public:
	RecordTabHandler();
	~RecordTabHandler();

	/**
	 * \fn	void RecordTabHandler::onCreate();
	 *
	 * \brief	Creates the user interface
	 */

	void onCreate();

	/**
	 * \fn	void RecordTabHandler::setSharedRecordingConfiguration(SharedRecordingConfiguration recordingConfiguration);
	 *
	 * \brief	Sets shared recording configuration.
	 *
	 * \param	recordingConfiguration	The recording configuration.
	 */
	void setSharedRecordingConfiguration(SharedRecordingConfiguration recordingConfiguration);

	/**
	* \fn	void RecordTabHandler::setSharedImageRecordingConfiguration(SharedImageRecordingConfiguration imageRecordingConfiguration);
	*
	* \brief	Sets shared image recording configuration.
	*
	* \param	imageRecordingConfiguration		The image recording configuration.
	*/
	void setSharedImageRecordingConfiguration(SharedImageRecordingConfiguration imageRecordingConfiguration);

		/**
	* \fn	void RecordTabHandler::setSharedImageRecordingConfiguration(SharedImageRecordingConfiguration imageRecordingConfiguration);
	*
	* \brief	Sets shared image recording configuration.
	*
	* \param	imageRecordingConfiguration		The image recording configuration.
	*/
	void setSharedImageRecordingConfigurationForKinectV1(SharedImageRecordingConfiguration imageRecordingConfiguration);

	void setSharedStringStringRecordingConfiguration(SharedStringFileRecordingConfiguration stringFileRecordingConfiguration);

	void setSharedCommonConfiguration(SharedCommonConfiguration commonConfiguration);

	SharedCommonConfiguration getSharedCommonConfiguration();

	/**
	 * \fn	SharedRecordingConfiguration RecordTabHandler::getRecordConfiguration();
	 *
	 * \brief	Getter for the record configuration.
	 *
	 * \return	The record configuration.
	 */
	SharedRecordingConfiguration getRecordConfiguration();

	/**
	* \fn	SharedImageRecordingConfiguration RecordTabHandler::getImageRecordConfiguration();
	*
	* \brief	Getter for the image record configuration.
	*
	* \return	The image record configuration.
	*/
	SharedImageRecordingConfiguration getImageRecordConfiguration();

	/**
	 * \fn	bool RecordTabHandler::isColorEnabled();
	 *
	 * \brief	Queries if a color recording is enabled.
	 *
	 * \return	true if a color is enabled, false if not.
	 */
	bool isColorEnabled();

	/**
	* \fn	bool RecordTabHandler::isCenterEnabled();
	*
	* \brief	Queries if a centered recording is enabled.
	*
	* \return	true if a centered is enabled, false if not.
	*/
	bool isCenterEnabled();

	/**
	 * \fn	void RecordTabHandler::recordingStopped();
	 *
	 * \brief	Callback: Recording has stopped.
	 */
	void recordingStopped();

	/**
	 * \fn	bool RecordTabHandler::isRecording();
	 *
	 * \brief	Query if the RecordTab is currently recording.
	 *
	 * \return	true if recording, false if not.
	 */
	bool isRecording();

	/**
	 * \fn	void RecordTabHandler::updateWriterStatus(RecordCloudType type, std::wstring status);
	 *
	 * \brief	Updates the writer status with the provided status.
	 *
	 * \param	type  	The type of the writer whose status has changed.
	 * \param	status	The new status.
	 */
	void updateWriterStatus(RecordCloudType type, std::wstring status);

	void updateWriterStatus(StringFileRecordType recordType, std::wstring status);


	/**
	* \fn	void RecordTabHandler::updateWriterStatus(ImageRecordType type, std::wstring status);
	*
	* \brief	Updates the writer status with the provided status.
	*
	* \param	type  	The type of the writer whose status has changed.
	* \param	status	The new status.
	*/
	void updateWriterStatus(ImageRecordType type, std::wstring status);

	/** \brief	The fps limit changed signal. */
	boost::signal<void(int)> fpsLimitUpdated;

		/** \brief	enbale aligment of kinect v1 signal. */
	boost::signal<void(bool)> kinectV1AlignmentEnable;

	/** \brief	The color configuration changed signal: color is now enabled or not. */
	boost::signal<void(bool)> colorConfigurationChanged;

	/** \brief	The color configuration changed signal: color is now enabled or not. */
	boost::signal<void(bool)> centerConfigurationChanged;

	/** \brief	The start writing signal. */
	boost::signal<void(bool, SharedRecordingConfiguration, SharedImageRecordingConfiguration,SharedStringFileRecordingConfiguration,SharedImageRecordingConfiguration)> startWriting;

		/** \brief	The start writing signal. */
	boost::signal<void(bool, SharedRecordingConfiguration, SharedImageRecordingConfiguration)> startKinectV1Writing;


	/** \brief	The stop writing signal. */
	boost::signal<void(bool, SharedRecordingConfiguration, SharedImageRecordingConfiguration,SharedStringFileRecordingConfiguration,SharedImageRecordingConfiguration)> stopWriting;


	/** \brief	The fps limit changed signal. */
	boost::signal<void(KinectV1ImageRecordType)> v1ShowOptChanged;

		/** \brief	The fps limit changed signal. */
	boost::signal<void(int)> v1ShowResolutionChanged;
	boost::signal<void(KinectV1ImageRecordType,int)> v1RecordingResolutionChanged;

	/**
	* \fn	void RecordTabHandler::recordPathChanged(RecordCloudType type);
	*
	* \brief	Callback: Record path has changed.
	*
	* \param	type	The type of the Cloud which path has changed.
	*/
	void recordPathChanged(RecordCloudType type);

	/**
	* \fn	void RecordTabHandler::recordPathChanged(ImageRecordType type);
	*
	* \brief	Callback: Record path has changed.
	*
	* \param	type	The type of the image which path has changed.
	*/
	void recordPathChanged(ImageRecordType type);

	void recordPathChanged(StringFileRecordType type);


	/**
	 * \fn	void RecordTabHandler::recordConfigurationStatusChanged(RecordCloudType type, bool newState);
	 *
	 * \brief	Record configuration status changed.
	 *
	 * \param	type		The type of the record configuration which has changed.
	 * \param	newState	true if the configuration is now enabled, otherwise it is disabled.
	 */
	void recordConfigurationStatusChanged(RecordCloudType type, bool newState);

	/**
	* \fn	void RecordTabHandler::recordConfigurationStatusChanged(ImageRecordType type, bool newState);
	*
	* \brief	Record configuration status changed.
	*
	* \param	type		The type of the record configuration which has changed.
	* \param	newState	true if the configuration is now enabled, otherwise it is disabled.
	*/
	void recordConfigurationStatusChanged(ImageRecordType type, bool newState);

	void recordConfigurationStatusChanged(StringFileRecordType type, bool newState);


	void setKinectEnableOpt(bool v1Enabled, bool v2Enabled);
private:

	/**
	 * \fn	void RecordTabHandler::checkRecordingConfigurationPossible();
	 *
	 * \brief	Check recording configuration possible now and adjusts the UI accordingly.
	 */
	void checkRecordingConfigurationPossible();

	/**
	 * \fn	void RecordTabHandler::setupRecording();
	 *
	 * \brief	Sets the file paths and timestamp of the recording configurations.
	 */
	void setupRecording();

	void onSelectionChanged(WPARAM wParam, LPARAM handle);
	void onButtonClicked(WPARAM wParam, LPARAM handle);
	void onEditBoxeChanged(WPARAM wParam, LPARAM handle);
	void movieShowOptWindosOfV1();

	/**
	 * \fn	void RecordTabHandler::setRecording(bool enable);
	 *
	 * \brief	Sets the recording state and updates the UI.
	 *
	 * \param	enable	true to enable, false to disable.
	 */

	void setRecording(bool enable);

	/**
	 * \fn	void RecordTabHandler::setColorEnabled(bool enable);
	 *
	 * \brief	Enables or disables the color and notfies listners about that.
	 * 			(e.g. Windows Application to change the rendered Point Cloud)
	 *
	 * \param	enable	true to enable, false to disable.
	 */
	void setColorEnabled(bool enable);

	/**
	* \fn	void RecordTabHandler::setCeterEnabled(bool enable);
	*
	* \brief	Enables or disables the centering of recorded Clouds and notfies listners about that.
	 * 			(e.g. Windows Application to change the recorded Point Cloud)
	*
	* \param	enable	true to enable, false to disable.
	*/
	void setCeterEnabled(bool enable);

	/**
	 * \fn	void RecordTabHandler::updateFrameLimit();
	 *
	 * \brief	Updates the frame limit according to the new values in the UI
	 * 			which has changed by the user.
	 */

	void updateFrameLimit();

	/**
	* \fn	void RecordTabHandler::updateFPSLimit();
	*
	* \brief	Updates the fps limit according to the new values in the UI
	* 			which has changed by the user.
	*/

	void updateFPSLimit();


	/** \brief	The recording configuration. */
	SharedCommonConfiguration m_commonConfiguration;

	/** \brief	The recording configuration. */
	SharedRecordingConfiguration m_recordingConfiguration;

	/** \brief	The image recording configuration. */
	SharedImageRecordingConfiguration m_imageRecordingConfiguration;

		/** \brief	The image recording configuration. */
	SharedImageRecordingConfiguration m_imageRecordingConfigurationForKinectV1;

		/** \brief	The image recording configuration. */
	SharedStringFileRecordingConfiguration m_KeyPointsRecordingConfiguration;

	/** \brief	true to enable, false to disable the color. */
	bool m_colorEnabled;


	/** \brief	true to enable, false to disable the centering. */
	bool m_centerEnabled;


	/** \brief	true if recording is running at the moment. */
	bool m_isRecording;

	kinectEnabledOpt m_KinectEnableOpt; // 0 KinectV2 1 KinectV1 2 both
};

