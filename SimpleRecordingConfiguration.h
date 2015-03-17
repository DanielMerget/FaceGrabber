#pragma once
#include "stdafx.h"

#include "IRecordingConfiguration.h"
#include <memory>
class SimpleRecordingConfiguration : public IRecordingConfiguration
{
public:
	SimpleRecordingConfiguration();
	~SimpleRecordingConfiguration();

	RecordingFileFormat getRecordFileFormat();
	std::string getFullRecordingPathString();
	std::string getFileNameString();

	void setRecordFileFormat(RecordingFileFormat);
	void setFullRecordingPathString(std::string);
	void setFileNameString(std::string);

	bool isRecordConfigurationValid();

	bool isRecordingDurationUnLimited();
	int getMaxNumberOfFrames();
	void setMaxNumberOfFrames(int newMaxNumberOfFrames);

	int getThreadCountToStart();
	void setThreadCountToStart(int);
private:
	std::string			m_fileName;
	std::string			m_fullRecordingPath;
	RecordingFileFormat m_recordingFileFormat;
	int					m_maximumNumberOfFrames;
	int					m_threadsCount;
};
typedef std::shared_ptr<SimpleRecordingConfiguration> SimpleRecordingConfigurationPtr;

