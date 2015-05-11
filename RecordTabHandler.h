#pragma once
#include "stdafx.h"

#include "MessageRouterHelper.h"
#include <windowsx.h>
#include "resource.h"
#include "RecordingConfiguration.h"
#include <memory>
#include <boost/signals.hpp>

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
	 * \fn	SharedRecordingConfiguration RecordTabHandler::getRecordConfiguration();
	 *
	 * \brief	Getter for the record configuration.
	 *
	 * \return	The record configuration.
	 */
	SharedRecordingConfiguration getRecordConfiguration();

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

	/** \brief	The color configuration changed signal: color is noew enabled or not. */
	boost::signal<void(bool)> colorConfigurationChanged;

	/** \brief	The color configuration changed signal: color is noew enabled or not. */
	boost::signal<void(bool)> centerConfigurationChanged;

	/** \brief	The start writing signal. */
	boost::signal<void(bool, SharedRecordingConfiguration)> startWriting;

	/** \brief	The stop writing signal. */
	boost::signal<void(bool, SharedRecordingConfiguration)> stopWriting;

	/**
	* \fn	void RecordTabHandler::recordPathChanged(RecordCloudType type);
	*
	* \brief	Callback: Record path has changed.
	*
	* \param	type	The type of the Cloud which path has changed.
	*/
	void recordPathChanged(RecordCloudType type);

	/**
	 * \fn	void RecordTabHandler::recordConfigurationStatusChanged(RecordCloudType type, bool newState);
	 *
	 * \brief	Record configuration status changed.
	 *
	 * \param	type		The type of the record configuration which has changed.
	 * \param	newState	true if the configuration is now enabled, otherwise it is disabled.
	 */
	void recordConfigurationStatusChanged(RecordCloudType type, bool newState);
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


	/** \brief	The recording configuration. */
	SharedRecordingConfiguration m_recordingConfiguration;


	/** \brief	true to enable, false to disable the color. */
	bool m_colorEnabled;


	/** \brief	true to enable, false to disable the centering. */
	bool m_centerEnabled;


	/** \brief	true if recording is running at the moment. */
	bool m_isRecording;
};

