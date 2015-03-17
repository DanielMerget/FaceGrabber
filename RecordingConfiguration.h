#pragma once
#include "stdafx.h"

#include <string>
#include "stdafx.h"
#include <boost/signal.hpp>
#include <boost/bind.hpp>
#include <boost/filesystem.hpp>
#include <atlstr.h>
#include <regex>

#include "IRecordingConfiguration.h"


class RecordingConfiguration : public IRecordingConfiguration{
#define UNLIMITED_FRAMES -1;
#define FRAMES_NOT_SET 0;

public:
	RecordingConfiguration();
	RecordingConfiguration(RecordCloudType cloudType, RecordingFileFormat format);

	RecordingConfiguration(RecordingConfiguration& recordingConfiguration);

	std::string getFileFormatFileExtension();
	static LPTSTR getFileFormatAsString(RecordingFileFormat fileFormat);

	void setDefaultFileName();

	LPTSTR getDefauldFileName();

	bool isRecordConfigurationValid();

	CString getFilePathCString();

	std::string getFilePath();

	bool isEnabled();

	RecordCloudType getRecordCloudType();

	RecordingFileFormat getRecordFileFormat();

	CString getFileNameCString();

	bool isRecordingDurationUnLimited();

	void setMaxNumberOfFrames(int newMaxNumberOfFrames);

	int getMaxNumberOfFrames();

	std::string getFileNameString();
	LPTSTR getFileName();

	void setFileName(LPTSTR fileName);
	void setFilePath(std::string filePath);

	void setFilePath(LPTSTR filePath);

	void setEnabled(bool enabled);

	void setFileFormat(RecordingFileFormat fileFormat);

	void setTimeStampFolderName(CString folderName);

	CString getTimeStampFolderName();


	static CString getSubFolderNameForCloudType(RecordCloudType cloudType);


	CString getFullRecordingPath();

	std::string getFullRecordingPathString();

	static CString getFullRecordingPathForCloudType(RecordCloudType cloudType, CString outputFolder, CString timeStampFolderName);

	boost::signal<void(RecordCloudType, bool)> recordConfigurationStatusChanged;
	boost::signal<void(RecordCloudType)>	recordPathOrFileNameChanged;

	int getThreadCountToStart();

	void setThreadCountToStart(int threadsCount);
private:
	std::vector<std::string> m_foundCloudFiles;

	//outputfolder/timestamp/cloudtype/filename.fileformat

	CString					m_outputFolder;
	CString					m_timeStampFolderName;
	CString					m_fileName;
	RecordCloudType			m_cloudType;
	RecordingFileFormat		m_fileFormat;
	int						m_maxNumberOfFrames;
	bool					m_enabled;
	int						m_threadsCount;
};
typedef std::shared_ptr<RecordingConfiguration> RecordingConfigurationPtr;
typedef std::vector<RecordingConfigurationPtr> SharedRecordingConfiguration;
