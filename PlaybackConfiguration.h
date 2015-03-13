#pragma once

#include <string>
#include "stdafx.h"
#include <boost/signal.hpp>
#include <boost/bind.hpp>
#include <atlstr.h>
#include "RecordingConfiguration.h"
#include <regex>
#include <algorithm>
#include <iostream>


struct CloudFile{
	//RecordingFileFormat format;
	std::string fileName;
	std::string fullFilePath;
};
class PlaybackConfiguration{

public:
	PlaybackConfiguration();

	PlaybackConfiguration(RecordingConfiguration& recordConfiguration);

	PlaybackConfiguration(PlaybackConfiguration& playbackConfigurationToCopy);

	PlaybackConfiguration& operator=(const PlaybackConfiguration& rhs);

	bool operator==(PlaybackConfiguration& playbackToCompareWith);

	bool isPlaybackConfigurationValid();

	CString getFilePathCString();

	bool isEnabled();

	RecordingFileFormat getRecordFileFormat();

	LPTSTR getFileName();

	void setFullFilePath(CString filePath);

	void setEnabled(bool enabled);

	RecordCloudType getRecordCloudType();

	CString getFirstPlaybackFile();

	int getCloudFilesToPlayCount();

	std::vector<CloudFile> getCloudFilesToPlay();

	void sortCloudFilesForPlayback();

	boost::signal<void(void)> playbackConfigurationChanged;

	void setWasFullPlayed();
	bool wasFullPlayed();
private:

	std::string getFilePath();

	void findFilesAtPath();


	static int extractCloudCountFromString(std::string input);

	static bool sortByIntegerEnding(const CloudFile& cloudFile1, const CloudFile cloudFile2);

	std::vector<CloudFile>	m_foundCloudFiles;
	CString						m_filePath;
	RecordingFileFormat			m_fileFormat;
	RecordCloudType				m_cloudType;
	CString						m_fileName;
	bool						m_enabled;
	bool						m_wasFullPlayed;

};

typedef std::shared_ptr<PlaybackConfiguration> PlaybackConfigurationPtr;
typedef std::vector<PlaybackConfigurationPtr> SharedPlaybackConfiguration;