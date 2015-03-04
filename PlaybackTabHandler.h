#pragma once
#include "resource.h"
#include "stdafx.h"
#include <atlstr.h>
#include <Windowsx.h>

#include "RecordTabHandler.h"
#include <strsafe.h>
#include <memory>
#include "PlaybackConfiguration.h"
#include <boost/signal.hpp>

class PlaybackTabHandler
{
public:
	PlaybackTabHandler();
	~PlaybackTabHandler();


	LRESULT CALLBACK		DlgProcTab(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam);
	static LRESULT CALLBACK	MessageRouterTab(HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam);;


	void setSharedRecordingConfiguration(SharedRecordingConfiguration recordingConfiguration);
	void playbackConfigurationChanged();
	void resetUIElements();
	void playbackStopped();

	boost::signal<void(SharedPlaybackConfiguration)> startPlayback;
	boost::signal<void(void)> stopPlayback;
private:
	void setPlaybackStatus(bool enable);
	void onCreate(WPARAM wParam, LPARAM);
	void checkPlayBackPossible();
	void processUIMessage(WPARAM wParam, LPARAM);
	void updateUIConfiguration();

	void onSelectionChanged(WPARAM wParam, LPARAM handle);
	void onButtonClicked(WPARAM wParam, LPARAM handle);
	void onEditBoxeChanged(WPARAM wParam, LPARAM handle);
	
	
	//SharedRecordingConfiguration m_recordingConfiguration;
	SharedPlaybackConfiguration m_playbackConfiguration;
	bool m_isPlaybackRunning;
	HWND m_hWnd;
};

