#include "stdafx.h"
#include "StringFileRecordingConfiguration.h"

StringFileRecordingConfiguration::StringFileRecordingConfiguration() : m_outputFolder(), m_enabled(false)
{}


StringFileRecordingConfiguration::StringFileRecordingConfiguration(StringFileRecordType recordType, StringFileRecordingFileFormat format) :
m_outputFolder(),
m_enabled(false),
m_recordType(recordType),
m_fileFormat(format)
{
	m_maxNumberOfFrames = UNLIMITED_FRAMES;
	setDefaultFileName();
}


CString StringFileRecordingConfiguration::getFileFormatAsString(StringFileRecordingFileFormat fileFormat)
{
	switch (fileFormat)
	{
	case TXT:
		return CString(L"txt");
	
	case STRING_FILE_RECORD_FILE_FORMAT_COUNT:
		return CString(L"ERROR");
	default:
		return CString(L"UNKNOWN_FILE FORMAT");
		break;
	}
}



void StringFileRecordingConfiguration::setDefaultFileName()
{
	m_fileName = getDefaultFileName();
}

CString StringFileRecordingConfiguration::getDefaultFileName()
{
	switch (m_recordType){
	case FiveKeyPoints:
		return CString(L"Five_Points_");
	default:
		return CString(L"NONE");
	}
	return CString(L"");
}

bool StringFileRecordingConfiguration::isRecordConfigurationValid()
{
	if (!m_enabled){
		return true;
	}
	bool fileNameLengthExists = (wcslen(m_fileName) > 0);
	bool filePathSet = !m_outputFolder.IsEmpty();
	bool limitCorrect = true;
	if (!isRecordingDurationUnLimited()){
		limitCorrect &= (m_maxNumberOfFrames > 0);
	}
	return fileNameLengthExists && filePathSet && limitCorrect;
}

CString StringFileRecordingConfiguration::getFilePathCString()
{
	return m_outputFolder;
}

std::string StringFileRecordingConfiguration::getFilePath()
{
	CT2CA pszConvertedAnsiString(m_outputFolder);
	std::string strStd(pszConvertedAnsiString);
	return strStd;
}

bool StringFileRecordingConfiguration::isEnabled()
{
	return m_enabled;
}

StringFileRecordType StringFileRecordingConfiguration::getRecordType(){
	return m_recordType;
}

StringFileRecordingFileFormat StringFileRecordingConfiguration::getRecordFileFormat()
{
	return m_fileFormat;
}



CString StringFileRecordingConfiguration::getFileNameCString()
{
	return m_fileName;
}

bool StringFileRecordingConfiguration::isRecordingDurationUnLimited()
{
	return m_maxNumberOfFrames == UNLIMITED_FRAMES;
}


void StringFileRecordingConfiguration::setMaxNumberOfFrames(int newMaxNumberOfFrames)
{
	m_maxNumberOfFrames = newMaxNumberOfFrames;
	recordPathOrFileNameChanged(m_recordType);
}

int StringFileRecordingConfiguration::getMaxNumberOfFrames()
{
	return m_maxNumberOfFrames;
}

std::string StringFileRecordingConfiguration::getFileNameString()
{
	CT2CA pszConvertedAnsiString(m_fileName);
	std::string strStd(pszConvertedAnsiString);
	return strStd;
}

void StringFileRecordingConfiguration::setFileName(CString fileName)
{
	m_fileName = fileName;
	recordPathOrFileNameChanged(m_recordType);
}

void StringFileRecordingConfiguration::setFilePath(CString filePath)
{
	m_outputFolder = CString(filePath);
	recordPathOrFileNameChanged(m_recordType);
}

int StringFileRecordingConfiguration::getThreadCountToStart()
{
	return m_threadsCount;
}

void StringFileRecordingConfiguration::setThreadCountToStart(int threadsCount)
{
	m_threadsCount = threadsCount;
}

void StringFileRecordingConfiguration::setEnabled(bool enabled)
{
	m_enabled = enabled;
	recordConfigurationStatusChanged(m_recordType, m_enabled);
}

void StringFileRecordingConfiguration::setFileFormat(StringFileRecordingFileFormat fileFormat)
{
	m_fileFormat = fileFormat;
}


void StringFileRecordingConfiguration::setTimeStampFolderName(CString folderName)
{
	m_timeStampFolderName = folderName;
}

CString StringFileRecordingConfiguration::getTimeStampFolderName()
{
	return m_timeStampFolderName;
}


CString StringFileRecordingConfiguration::getSubFolderNameForCloudType(StringFileRecordType recordType)
{
	switch (recordType)
	{
	case FiveKeyPoints:
		return L"FiveKeypoints";
	case STRING_FILE_RECORD_TYPE_COUNT:
		break;
	default:
		break;
	}
	return L"UKNOWN CLOUD TYPE";
}


CString StringFileRecordingConfiguration::getFullRecordingPath()
{
	auto result = getFullRecordingPathForRecordType(m_recordType, m_outputFolder, m_timeStampFolderName,m_kinectVersion);
	return result;
}


std::string StringFileRecordingConfiguration::getFullRecordingPathString()
{
	CString fullPath = getFullRecordingPath();
	CT2CA pszConvertedAnsiString(fullPath);
	std::string strStd(pszConvertedAnsiString);
	return strStd;
}

CString StringFileRecordingConfiguration::getFullRecordingPathForRecordType(StringFileRecordType recordType, CString outputFolder, CString timeStampFolderName,KinectVersionType kinectVersion)
{
	CString subFolder = getSubFolderNameForCloudType(recordType);
	CString fullPath;

	fullPath += outputFolder.GetString();

	auto outputEnding = fullPath.Right(1);
	if (outputEnding != L"\\"){
		fullPath += _T("\\");
	}
	if(kinectVersion == KinectVersionType::KinectV1)
	{
		fullPath += _T("KinectV1\\");
	}
	else if(kinectVersion== KinectVersionType::KinectV2)
	{
		fullPath += _T("KinectV2\\");
	}

	fullPath += timeStampFolderName.GetString();
	outputEnding = fullPath.Right(1);
	if (outputEnding != L"\\"){
		fullPath += _T("\\");
	}
	fullPath += subFolder;
	return fullPath;
}

void StringFileRecordingConfiguration::setKinectVersion(KinectVersionType	kinectVersion)
{
	m_kinectVersion = kinectVersion;
}