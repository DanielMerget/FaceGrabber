#include "stdafx.h"
#include "ImageRecordingConfiguration.h"

ImageRecordingConfiguration::ImageRecordingConfiguration() : m_outputFolder(), m_enabled(false)
{}


ImageRecordingConfiguration::ImageRecordingConfiguration(ImageRecordType imageType, ImageRecordingFileFormat format) :
m_outputFolder(),
m_enabled(false),
m_imageType(imageType),
m_fileFormat(format)
{
	m_maxNumberOfFrames = UNLIMITED_FRAMES;
	setDefaultFileName();
}


CString ImageRecordingConfiguration::getFileFormatAsString(ImageRecordingFileFormat fileFormat)
{
	switch (fileFormat)
	{
	case PNM_BINARY:
		return CString(L"pnm binary");
	case PNG:
		return CString(L"png");
	case IMAGE_RECORD_FILE_FORMAT_COUNT:
		return CString(L"ERROR");
	default:
		return CString(L"UNKNOWN_FILE FORMAT");
		break;
	}
}

void ImageRecordingConfiguration::setDefaultFileName()
{
	m_fileName = getDefaultFileName();
}

CString ImageRecordingConfiguration::getDefaultFileName()
{
	switch (m_imageType){
	case KinectColorRaw:
		return CString(L"Kinect_Color_Raw");
	case KinectDepthRaw:
		return CString(L"Kinect_Depth_Raw");
	default:
		return CString(L"Image");
	}
	return CString(L"");
}

bool ImageRecordingConfiguration::isRecordConfigurationValid()
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

CString ImageRecordingConfiguration::getFilePathCString()
{
	return m_outputFolder;
}

std::string ImageRecordingConfiguration::getFilePath()
{
	CT2CA pszConvertedAnsiString(m_outputFolder);
	std::string strStd(pszConvertedAnsiString);
	return strStd;
}

bool ImageRecordingConfiguration::isEnabled()
{
	return m_enabled;
}

ImageRecordType ImageRecordingConfiguration::getImageRecordType(){
	return m_imageType;
}

ImageRecordingFileFormat ImageRecordingConfiguration::getRecordFileFormat()
{
	return m_fileFormat;
}

CString ImageRecordingConfiguration::getFileNameCString()
{
	return m_fileName;
}

bool ImageRecordingConfiguration::isRecordingDurationUnLimited()
{
	return m_maxNumberOfFrames == UNLIMITED_FRAMES;
}


void ImageRecordingConfiguration::setMaxNumberOfFrames(int newMaxNumberOfFrames)
{
	m_maxNumberOfFrames = newMaxNumberOfFrames;
	recordPathOrFileNameChanged(m_imageType);
}

int ImageRecordingConfiguration::getMaxNumberOfFrames()
{
	return m_maxNumberOfFrames;
}

std::string ImageRecordingConfiguration::getFileNameString()
{
	CT2CA pszConvertedAnsiString(m_fileName);
	std::string strStd(pszConvertedAnsiString);
	return strStd;
}

void ImageRecordingConfiguration::setFileName(CString fileName)
{
	m_fileName = fileName;
	recordPathOrFileNameChanged(m_imageType);
}

void ImageRecordingConfiguration::setFilePath(CString filePath)
{
	m_outputFolder = CString(filePath);
	recordPathOrFileNameChanged(m_imageType);
}

int ImageRecordingConfiguration::getThreadCountToStart()
{
	return m_threadsCount;
}

void ImageRecordingConfiguration::setThreadCountToStart(int threadsCount)
{
	m_threadsCount = threadsCount;
}

void ImageRecordingConfiguration::setEnabled(bool enabled)
{
	m_enabled = enabled;
	recordConfigurationStatusChanged(m_imageType, m_enabled);
}

void ImageRecordingConfiguration::setFileFormat(ImageRecordingFileFormat fileFormat)
{
	m_fileFormat = fileFormat;
}

void ImageRecordingConfiguration::setTimeStampFolderName(CString folderName)
{
	m_timeStampFolderName = folderName;
}

CString ImageRecordingConfiguration::getTimeStampFolderName()
{
	return m_timeStampFolderName;
}


CString ImageRecordingConfiguration::getSubFolderNameForImageType(ImageRecordType imageType)
{
	switch (imageType)
	{
	case KinectColorRaw:
		return L"KinectColorRaw";
	case KinectDepthRaw:
		return L"KinectDepthRaw";
	case IMAGE_RECORD_TYPE_COUNT:
		break;
	default:
		break;
	}
	return L"UKNOWN IMAGE TYPE";
}


CString ImageRecordingConfiguration::getFullRecordingPath()
{
	auto result = getFullRecordingPathForImageType(m_imageType, m_outputFolder, m_timeStampFolderName);
	return result;
}


std::string ImageRecordingConfiguration::getFullRecordingPathString()
{
	CString fullPath = getFullRecordingPath();
	CT2CA pszConvertedAnsiString(fullPath);
	std::string strStd(pszConvertedAnsiString);
	return strStd;
}

CString ImageRecordingConfiguration::getFullRecordingPathForImageType(ImageRecordType imageType, CString outputFolder, CString timeStampFolderName)
{
	CString subFolder = getSubFolderNameForImageType(imageType);
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

