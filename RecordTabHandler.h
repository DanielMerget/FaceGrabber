#pragma once
#include <windowsx.h>
#include "stdafx.h"
#include "resource.h"
#include "RecordingConfiguration.h"


#include <memory>
typedef std::shared_ptr<std::vector<RecordingConfiguration>> SharedRecordingConfiguration;
class RecordTabHandler
{
public:
	RecordTabHandler();
	~RecordTabHandler();

	void setSharedRecordingConfiguration(SharedRecordingConfiguration recordingConfiguration);
	
		   LRESULT CALLBACK	DlgProcTab(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam);
	static LRESULT CALLBACK	MessageRouterTab(HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam);;

	void onCreate(WPARAM wParam, LPARAM);
	void checkRecordingConfigurationPossible();
	void processUIMessage(WPARAM wParam, LPARAM);

	
	void onSelectionChanged(WPARAM wParam, LPARAM handle);
	void onButtonClicked(WPARAM wParam, LPARAM handle);
	void onEditBoxeChanged(WPARAM wParam, LPARAM handle);
	void recordPathChanged(RecordCloudType type);
	void recordConfigurationStatusChanged(RecordCloudType type, bool newState);

	SharedRecordingConfiguration m_recordingConfiguration;


	HWND m_hWnd;
};

