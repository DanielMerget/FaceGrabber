#pragma once
#include "resource.h"
#include "stdafx.h"
#include <atlstr.h>
#include <Windowsx.h>

#include "RecordTabHandler.h"
#include <strsafe.h>
#include <memory>

class PlaybackTabHandler
{
public:
	PlaybackTabHandler();
	~PlaybackTabHandler();


	LRESULT CALLBACK		DlgProcTab(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam);
	static LRESULT CALLBACK	MessageRouterTab(HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam);;


	void setSharedRecordingConfiguration(SharedRecordingConfiguration recordingConfiguration);
private:
	void onCreate(WPARAM wParam, LPARAM);
	void checkPlayBackPossible();
	void processUIMessage(WPARAM wParam, LPARAM);


	void onSelectionChanged(WPARAM wParam, LPARAM handle);
	void onButtonClicked(WPARAM wParam, LPARAM handle);
	void onEditBoxeChanged(WPARAM wParam, LPARAM handle);
	
	
	SharedRecordingConfiguration m_recordingConfiguration;

	HWND m_hWnd;
};

