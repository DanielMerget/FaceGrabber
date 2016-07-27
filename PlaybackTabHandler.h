#pragma once
#include "stdafx.h"

#include "resource.h"
#include <atlstr.h>
#include <Windowsx.h>
#include "MessageRouterHelper.h"
#include "RecordTabHandler.h"
#include <strsafe.h>
#include <memory>
#include "PlaybackConfiguration.h"
#include <boost/signal.hpp>

/**
 * \class	PlaybackTabHandler
 *
 * \brief	The Class handling all events of the user to configure the PlaybackConfiguration.
 * 			If requested by the user, the playback is triggered by fireing the startPlayback,
 * 			stopPlayback signals etc.
 */

class PlaybackTabHandler : public MessageRouterHelper
{
public:
	PlaybackTabHandler();
	~PlaybackTabHandler();

	/**
	 * \fn	void PlaybackTabHandler::setSharedRecordingConfiguration(SharedRecordingConfiguration recordingConfiguration);
	 *
	 * \brief	Sets shared recording configuration to preset values of the playback tab with the last recorded configuration
	 *
	 * \param	recordingConfiguration	The recording configuration.
	 */

	void setSharedRecordingConfiguration(SharedRecordingConfiguration recordingConfiguration);

	/**
	 * \fn	void PlaybackTabHandler::playbackConfigurationChanged();
	 *
	 * \brief	Callback if playback configuration changed.
	 */
	void playbackConfigurationChanged();

	/**
	 * \fn	void PlaybackTabHandler::resetUIElements();
	 *
	 * \brief	Resets the user interface elements.
	 */
	void resetUIElements();

	/**
	 * \fn	void PlaybackTabHandler::playbackStopped();
	 *
	 * \brief	Callback: Playback stopped by the user.
	 */
	void playbackStopped();


	/** \brief	signal to start the playback. */
	boost::signal<void(SharedPlaybackConfiguration,  bool)> startPlayback;

	/** \brief	signal to stop the playback. */
	boost::signal<void(void)> stopPlayback;

	/** \brief	signal to pause the playback. */
	boost::signal<void(bool)> pausePlayback;

	/** \brief	The fps limit changed signal. */
	boost::signal<void(int)> fpsLimitUpdated;

	/** \brief	The playback slider moved. */
	boost::signal<void(int)> playbackSliderMoved;

	/**
	 * \fn	void PlaybackTabHandler::updateReaderStatus(RecordCloudType type, std::wstring status);
	 *
	 * \brief	Callback: Updates status of the reader.
	 *
	 * \param	type  	The type of the played point cloud
	 * \param	status	The new status.
	 */
	void updateReaderStatus(RecordCloudType type, std::wstring status);

	/**
	 * \fn	bool PlaybackTabHandler::isPlaybackRunning();
	 *
	 * \brief	Query if this object is playback running.
	 *
	 * \return	true if playback running, false if not.
	 */
	bool isPlaybackRunning();

	/**
	* \fn	void RecordTabHandler::updatePlaybackSliderPos();
	*
	* \brief	Updates the position of the playback slider
	*/
	void updatePlaybackSliderPos(int pos);

	/**
	* \fn	void RecordTabHandler::updatePlaybackSliderRange();
	*
	* \brief	Updates the range of the playback slider
	*/
	void updatePlaybackSliderRange(int min, int max);
private:

	/**
	 * \fn	void PlaybackTabHandler::setPlaybackStatus(bool enable);
	 *
	 * \brief	Sets playback status.
	 *
	 * \param	enable	true to enable, false to disable.
	 */
	void setPlaybackStatus(bool enable);

	/**
	* \fn	void PlaybackTabHandler::setPlaybackPaused(bool enable);
	*
	* \brief	Sets playback paused.
	*
	* \param	enable	true to enable, false to disable.
	*/
	void setPlaybackPaused(bool enable);

	/**
	 * \fn	void PlaybackTabHandler::onCreate();
	 *
	 * \brief	Executes the creates the user interface with preset/standard values
	 */
	void onCreate();

	void onSliderChanged(WPARAM wParam, LPARAM handle);
	void onSelectionChanged(WPARAM wParam, LPARAM handle);
	void onButtonClicked(WPARAM wParam, LPARAM handle);
	void onEditBoxeChanged(WPARAM wParam, LPARAM handle);

	/**
	* \fn	void RecordTabHandler::updateFPSLimit();
	*
	* \brief	Updates the fps limit according to the new values in the UI
	* 			which has changed by the user.
	*/
	void updateFPSLimit();

	/** \brief	The playback configuration. */
	SharedPlaybackConfiguration m_playbackConfiguration;

	/** \brief	true if the Playback is running. */
	bool m_isPlaybackRunning;

	/** \brief	true if the Playback is paused. */
	bool m_isPlaybackPaused;

	/** \brief	true if the reading is done is single threaded. */
	bool m_isSingleThreadedReading;
};

