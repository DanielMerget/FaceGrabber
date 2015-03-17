#pragma once
#include "resource.h"
#include "stdafx.h"
#include <atlstr.h>
#include <Windowsx.h>
#include "MessageRouterHelper.h"
#include "RecordTabHandler.h"
#include <strsafe.h>
#include <memory>
#include "PlaybackConfiguration.h"
#include <boost/signal.hpp>

class PlaybackTabHandler : public MessageRouterHelper
{
public:
	PlaybackTabHandler();
	~PlaybackTabHandler();

	void setSharedRecordingConfiguration(SharedRecordingConfiguration recordingConfiguration);
	void playbackConfigurationChanged();
	void resetUIElements();
	void playbackStopped();

	boost::signal<void(SharedPlaybackConfiguration,  bool)> startPlayback;
	boost::signal<void(void)> stopPlayback;

	void updateReaderStatus(RecordCloudType type, std::wstring status);

	bool isPlaybackRunning();
private:
	void setPlaybackStatus(bool enable);
	void onCreate();
	void checkPlayBackPossible();
	void updateUIConfiguration();

	void onSelectionChanged(WPARAM wParam, LPARAM handle);
	void onButtonClicked(WPARAM wParam, LPARAM handle);
	void onEditBoxeChanged(WPARAM wParam, LPARAM handle);
	
	SharedPlaybackConfiguration m_playbackConfiguration;
	bool m_isPlaybackRunning;
	bool m_isSingleThreadedReading;
};

