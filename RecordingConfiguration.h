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
	
public:
	RecordingConfiguration() : m_outputFolder(), m_enabled(false)
	{}

	RecordingConfiguration(RecordingConfiguration&& recordConfiguration) :
		m_outputFolder(recordConfiguration.getFilePathCString()),
		m_cloudType(recordConfiguration.getRecordCloudType()),
		m_enabled(false),
		m_fileFormat(recordConfiguration.getRecordFileFormat()),
		m_fileName(recordConfiguration.getFileName())
	{
	}

	RecordingConfiguration(RecordCloudType cloudType, RecordingFileFormat format) : 
		m_outputFolder(),
		m_enabled(false),
		m_cloudType(cloudType),
		m_fileFormat(format)
	{
		setDefaultFileName(); 
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
		return fileNameLengthExists && filePathSet;
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
		//A std:string  using the char* constructor.
		//char ch[MAX_PATH];
		//char DefChar = ' ';
		//WideCharToMultiByte(CP_ACP, 0, filePath, -1, ch, MAX_PATH, &DefChar, NULL);
		//
		//m_filePath = std::string(ch);
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

	static std::string getSubFolderNameForCloudType(RecordCloudType cloudType)
	{
		switch (cloudType)
		{
		case HDFace:
			return "HDFace";
		case FaceRaw:
			return "FaceRaw";
		case FullDepthRaw:
			return "FullDepthRaw";
		case RECORD_CLOUD_TYPE_COUNT:
			break;
		default:
			break;
		}
		return "UKNOWN CLOUD TYPE";
	}

	static std::string getFullRecordingPathForCloudType(RecordCloudType cloudType, std::string filePath)
	{
		std::string subFolder = getSubFolderNameForCloudType(cloudType);

		std::stringstream fullPath;
		fullPath << filePath;
		fullPath << "\\";
		fullPath << subFolder;
		std::string result = fullPath.str();
		return result;
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
	bool					m_colourInformation;

	
	//LPTSTR					m_fileName;
	bool					m_enabled;
};

typedef std::vector<std::shared_ptr<RecordingConfiguration>> SharedRecordingConfiguration;