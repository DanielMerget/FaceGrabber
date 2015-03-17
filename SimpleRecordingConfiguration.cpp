#include "stdafx.h"
#include "SimpleRecordingConfiguration.h"


SimpleRecordingConfiguration::SimpleRecordingConfiguration() : m_threadsCount(4)
{
}


SimpleRecordingConfiguration::~SimpleRecordingConfiguration()
{

}


RecordingFileFormat SimpleRecordingConfiguration::getRecordFileFormat()
{
	return m_recordingFileFormat;
}

std::string SimpleRecordingConfiguration::getFullRecordingPathString()
{
	return m_fullRecordingPath;
}

std::string SimpleRecordingConfiguration::getFileNameString()
{
	return m_fileName;
}


void SimpleRecordingConfiguration::setRecordFileFormat(RecordingFileFormat recordingFormat)
{
	m_recordingFileFormat = recordingFormat;
}

void SimpleRecordingConfiguration::setFullRecordingPathString(std::string fullfileFormat)
{
	m_fullRecordingPath = fullfileFormat;
}

void SimpleRecordingConfiguration::setFileNameString(std::string fileName)
{
	m_fileName = fileName;
}

bool SimpleRecordingConfiguration::isRecordingDurationUnLimited()
{
	return false;
}
int SimpleRecordingConfiguration::getMaxNumberOfFrames()
{
	return m_maximumNumberOfFrames;
}

void SimpleRecordingConfiguration::setMaxNumberOfFrames(int newMaxNumberOfFrames)
{
	m_maximumNumberOfFrames = newMaxNumberOfFrames;
}

bool SimpleRecordingConfiguration::isRecordConfigurationValid()
{
	return !m_fullRecordingPath.empty() && !m_fileName.empty();
}

int SimpleRecordingConfiguration::getThreadCountToStart()
{
	return m_threadsCount;
}

void SimpleRecordingConfiguration::setThreadCountToStart(int threadsCount)
{
	m_threadsCount = threadsCount;
}