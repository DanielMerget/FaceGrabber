#pragma once
#include <string>
#include "stdafx.h"
#include <boost/signal.hpp>
#include <boost/bind.hpp>
#include <boost/filesystem.hpp>
#include <atlstr.h>
#include <regex>


enum RecordingFileFormat{
	PLY,
	PLY_BINARY,
	PCD,
	PCD_BINARY,
	RECORD_FILE_FORMAT_COUNT
};

enum RecordCloudType{
	HDFace,
	FaceRaw,
	FullDepthRaw,
	RECORD_CLOUD_TYPE_COUNT
};

class RecordingConfiguration{
#define UNLIMITED_FRAMES -1;
#define FRAMES_NOT_SET 0;
public:
	RecordingConfiguration() : m_outputFolder(), m_enabled(false)
	{}


	RecordingConfiguration(RecordCloudType cloudType, RecordingFileFormat format) : 
		m_outputFolder(),
		m_enabled(false),
		m_cloudType(cloudType),
		m_fileFormat(format)
	{
		m_maxNumberOfFrames = UNLIMITED_FRAMES;
		setDefaultFileName(); 
	}


	std::string getFileFormatFileExtension()
	{
		switch (m_fileFormat)
		{
		case PLY:
			return ".ply";
			break;
		case PLY_BINARY:
			return ".ply";
			break;
		case PCD:
			return ".pcd";
			break;
		case PCD_BINARY:
			return ".pcd";
			break;
		case RECORD_FILE_FORMAT_COUNT:
			break;
		default:
			break;
		}
		return ".UNKNOWN_FILE_FORMAT";
	}
	static LPTSTR getFileFormatAsString(RecordingFileFormat fileFormat)
	{
		switch (fileFormat)
		{
		case PLY:
			return L"ply";
		case PLY_BINARY:
			return L"ply binary";
		case PCD:
			return L"pcd";
		case PCD_BINARY:
			return L"pcd binary";
		case RECORD_FILE_FORMAT_COUNT:
			return L"ERROR";
		default:
			return L"UNKNOWN_FILE FORMAT";
			break;
		}
	}

	void setDefaultFileName()
	{
		m_fileName = getDefauldFileName();
	}

	LPTSTR getDefauldFileName()
	{
		switch (m_cloudType){
		case HDFace:
			return L"HD_Face";
		case FaceRaw:
			return L"Face_Raw";
		case FullDepthRaw:
			return L"Full_Raw_Depth";
		default:
			return L"Cloud";
		}
		return L"";
	}
	
	bool isRecordConfigurationValid()
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

	CString getFilePathCString()
	{
		return m_outputFolder;
	}

	std::string getFilePath()
	{
		CT2CA pszConvertedAnsiString(m_outputFolder);
		std::string strStd(pszConvertedAnsiString);
		return strStd;
	}

	bool isEnabled()
	{
		return m_enabled;
	}

	RecordCloudType getRecordCloudType(){
		return m_cloudType;
	}

	RecordingFileFormat getRecordFileFormat()
	{
		return m_fileFormat;
	}

	CString getFileNameCString()
	{
		return m_fileName;
	}

	bool isRecordingDurationUnLimited()
	{
		return m_maxNumberOfFrames == UNLIMITED_FRAMES;
	}

	
	void setMaxNumberOfFrames(int newMaxNumberOfFrames)
	{
		m_maxNumberOfFrames = newMaxNumberOfFrames;
		recordPathOrFileNameChanged(m_cloudType);
	}

	int getMaxNumberOfFrames()
	{
		return m_maxNumberOfFrames;
	}

	std::string getFileNameString()
	{
		CT2CA pszConvertedAnsiString(m_fileName);
		std::string strStd(pszConvertedAnsiString);
		return strStd;
	}

	LPTSTR getFileName()
	{
		return m_fileName.GetBuffer(0);
	}

	void setFileName(LPTSTR fileName)
	{
		m_fileName = fileName;
		recordPathOrFileNameChanged(m_cloudType);
	}

	void setFilePath(std::string filePath)
	{
		m_outputFolder = CString(filePath.c_str());
		recordPathOrFileNameChanged(m_cloudType);
	}

	void setFilePath(LPTSTR filePath)
	{
		m_outputFolder = CString(filePath);
		recordPathOrFileNameChanged(m_cloudType);
	}

	
	void setEnabled(bool enabled)
	{
		m_enabled = enabled;
		recordConfigurationStatusChanged(m_cloudType, m_enabled);
	}

	void setFileFormat(RecordingFileFormat fileFormat)
	{
		m_fileFormat = fileFormat;
	}

	void setTimeStampFolderName(CString folderName)
	{
		m_timeStampFolderName = folderName;
	}

	CString getTimeStampFolderName()
	{
		return m_timeStampFolderName;
	}

	
	static CString getSubFolderNameForCloudType(RecordCloudType cloudType)
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


	CString getFullRecordingPath()
	{
		auto result = getFullRecordingPathForCloudType(m_cloudType, m_outputFolder, m_timeStampFolderName);
		return result;
	}


	std::string getFullRecordingPathString()
	{
		CString fullPath = getFullRecordingPath();
		CT2CA pszConvertedAnsiString(fullPath);
		std::string strStd(pszConvertedAnsiString);
		return strStd;
	}

	static CString getFullRecordingPathForCloudType(RecordCloudType cloudType, CString outputFolder, CString timeStampFolderName)
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

	boost::signal<void(RecordCloudType, bool)> recordConfigurationStatusChanged;
	boost::signal<void(RecordCloudType)>	recordPathOrFileNameChanged;
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
};
typedef std::shared_ptr<RecordingConfiguration> RecordingConfigurationPtr;
typedef std::vector<RecordingConfigurationPtr> SharedRecordingConfiguration;
