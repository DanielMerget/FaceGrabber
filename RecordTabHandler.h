#pragma once
#include "MessageRouterHelper.h"
#include <windowsx.h>
#include "stdafx.h"
#include "resource.h"
#include "RecordingConfiguration.h"
#include <memory>
#include <boost/signals.hpp>


class RecordTabHandler : public MessageRouterHelper
{
public:
	RecordTabHandler();
	~RecordTabHandler();

	void setSharedRecordingConfiguration(SharedRecordingConfiguration recordingConfiguration);

	void onCreate();
	void checkRecordingConfigurationPossible();

	SharedRecordingConfiguration getRecordConfiguration();
	void recordPathChanged(RecordCloudType type);
	void recordConfigurationStatusChanged(RecordCloudType type, bool newState);

	boost::signal<void(bool)> colorConfigurationChanged;
	boost::signal<void(bool)> startWriting;
	boost::signal<void(bool)> stopWriting;

	bool isColorEnabled();

	void recordingStopped();
	
	bool isRecording();

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
};

