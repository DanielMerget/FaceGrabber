#include "stdafx.h"
#include "RecordingConfiguration.h"

RecordingConfiguration::RecordingConfiguration() : m_outputFolder(), m_enabled(false)
{}


RecordingConfiguration::RecordingConfiguration(RecordCloudType cloudType, RecordingFileFormat format) :
m_outputFolder(),
m_enabled(false),
m_cloudType(cloudType),
m_fileFormat(format)
{
	m_maxNumberOfFrames = UNLIMITED_FRAMES;
	setDefaultFileName();
}


CString RecordingConfiguration::getFileFormatAsString(RecordingFileFormat fileFormat)
{
	switch (fileFormat)
	{
	case PLY:
		return CString(L"ply");
	case PLY_BINARY:
		return CString(L"ply binary");
	case PCD:
		return L"pcd";
	case PCD_BINARY:
		return CString(L"pcd binary");
	case RECORD_FILE_FORMAT_COUNT:
		return CString(L"ERROR");
	default:
		return CString(L"UNKNOWN_FILE FORMAT");
		break;
	}
}

void RecordingConfiguration::setDefaultFileName()
{
	m_fileName = getDefauldFileName();
}

CString RecordingConfiguration::getDefauldFileName()
{
	switch (m_cloudType){
	case HDFace:
		return CString(L"HD_Face");
	case FaceRaw:
		return CString(L"Face_Raw");
	case FullDepthRaw:
		return CString(L"Full_Raw_Depth");
	default:
		return CString(L"Cloud");
	}
	return CString(L"");
}

bool RecordingConfiguration::isRecordConfigurationValid()
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

CString RecordingConfiguration::getFilePathCString()
{
	return m_outputFolder;
}

std::string RecordingConfiguration::getFilePath()
{
	CT2CA pszConvertedAnsiString(m_outputFolder);
	std::string strStd(pszConvertedAnsiString);
	return strStd;
}

bool RecordingConfiguration::isEnabled()
{
	return m_enabled;
}

RecordCloudType RecordingConfiguration::getRecordCloudType(){
	return m_cloudType;
}

RecordingFileFormat RecordingConfiguration::getRecordFileFormat()
{
	return m_fileFormat;
}

CString RecordingConfiguration::getFileNameCString()
{
	return m_fileName;
}

bool RecordingConfiguration::isRecordingDurationUnLimited()
{
	return m_maxNumberOfFrames == UNLIMITED_FRAMES;
}


void RecordingConfiguration::setMaxNumberOfFrames(int newMaxNumberOfFrames)
{
	m_maxNumberOfFrames = newMaxNumberOfFrames;
	recordPathOrFileNameChanged(m_cloudType);
}

int RecordingConfiguration::getMaxNumberOfFrames()
{
	return m_maxNumberOfFrames;
}

std::string RecordingConfiguration::getFileNameString()
{
	CT2CA pszConvertedAnsiString(m_fileName);
	std::string strStd(pszConvertedAnsiString);
	return strStd;
}

LPTSTR RecordingConfiguration::getFileName()
{
	return m_fileName.GetBuffer(0);
}

void RecordingConfiguration::setFileName(LPTSTR fileName)
{
	m_fileName = fileName;
	recordPathOrFileNameChanged(m_cloudType);
}

void RecordingConfiguration::setFilePath(std::string filePath)
{
	m_outputFolder = CString(filePath.c_str());
	recordPathOrFileNameChanged(m_cloudType);
}

void RecordingConfiguration::setFilePath(LPTSTR filePath)
{
	m_outputFolder = CString(filePath);
	recordPathOrFileNameChanged(m_cloudType);
}

int RecordingConfiguration::getThreadCountToStart()
{
	return m_threadsCount;
}

void RecordingConfiguration::setThreadCountToStart(int threadsCount)
{
	m_threadsCount = threadsCount;
}

void RecordingConfiguration::setEnabled(bool enabled)
{
	m_enabled = enabled;
	recordConfigurationStatusChanged(m_cloudType, m_enabled);
}

void RecordingConfiguration::setFileFormat(RecordingFileFormat fileFormat)
{
	m_fileFormat = fileFormat;
}

void RecordingConfiguration::setTimeStampFolderName(CString folderName)
{
	m_timeStampFolderName = folderName;
}

CString RecordingConfiguration::getTimeStampFolderName()
{
	return m_timeStampFolderName;
}


CString RecordingConfiguration::getSubFolderNameForCloudType(RecordCloudType cloudType)
{
	switch (cloudType)
	{
	case HDFace:
		return L"HDFace";
	case FaceRaw:
		return L"FaceRaw";
	case FullDepthRaw:
		return L"FullDepthRaw";
	case RECORD_CLOUD_TYPE_COUNT:
		break;
	default:
		break;
	}
	return L"UKNOWN CLOUD TYPE";
}


CString RecordingConfiguration::getFullRecordingPath()
{
	auto result = getFullRecordingPathForCloudType(m_cloudType, m_outputFolder, m_timeStampFolderName);
	return result;
}


std::string RecordingConfiguration::getFullRecordingPathString()
{
	CString fullPath = getFullRecordingPath();
	CT2CA pszConvertedAnsiString(fullPath);
	std::string strStd(pszConvertedAnsiString);
	return strStd;
}

CString RecordingConfiguration::getFullRecordingPathForCloudType(RecordCloudType cloudType, CString outputFolder, CString timeStampFolderName)
{
	CString subFolder = getSubFolderNameForCloudType(cloudType);
	CString fullPath;

	fullPath += outputFolder.GetString();
	auto outputEnding = fullPath.Right(1);
	if (outputEnding != L"\\"){
		fullPath += _T("\\");
	}
	fullPath += timeStampFolderName.GetString();
	outputEnding = fullPath.Right(1);
	if (outputEnding != L"\\"){
		fullPath += _T("\\");
	}
	fullPath += subFolder;
	return fullPath;
}

