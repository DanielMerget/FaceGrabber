#pragma once
#include <windowsx.h>
#include "stdafx.h"
#include "resource.h"
#include "RecordingConfiguration.h"
#include <memory>
#include <boost/signals.hpp>


class RecordTabHandler
{
public:
	RecordTabHandler();
	~RecordTabHandler();

	void setSharedRecordingConfiguration(SharedRecordingConfiguration recordingConfiguration);
	//BOOL CALLBACK EnumChildProc(HWND hwnd, LPARAM lParam);
		   LRESULT CALLBACK	DlgProcTab(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam);
	static LRESULT CALLBACK	MessageRouterTab(HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam);;

	void onCreate(WPARAM wParam, LPARAM);
	void checkRecordingConfigurationPossible();
	void processUIMessage(WPARAM wParam, LPARAM);

	SharedRecordingConfiguration getRecordConfiguration();
	void recordPathChanged(RecordCloudType type);
	void recordConfigurationStatusChanged(RecordCloudType type, bool newState);

	boost::signal<void(bool)> colorConfigurationChanged;
	boost::signal<void(bool)> startWriting;
	boost::signal<void(bool)> stopWriting;

	bool isColorEnabled();

	void recordingStopped();
	
	void updateWriterStatus(RecordCloudType type, std::wstring status);
private:
	void setupRecording();
	void onSelectionChanged(WPARAM wParam, LPARAM handle);
	void onButtonClicked(WPARAM wParam, LPARAM handle);
	void onEditBoxeChanged(WPARAM wParam, LPARAM handle);
	void setRecording(bool enable);

	void setColorEnabled(bool enable);
	void updateFrameLimit();
	SharedRecordingConfiguration m_recordingConfiguration;

	bool m_colorEnabled;
	bool m_isRecording;
	HWND m_hWnd;
};

