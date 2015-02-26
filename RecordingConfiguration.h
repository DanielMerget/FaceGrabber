#include <string>
#include "stdafx.h"
#include <boost/signal.hpp>
#include <boost/bind.hpp>
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
	RecordingConfiguration() : m_filePath(), m_enabled(false)
	{}

	RecordingConfiguration(RecordingConfiguration&& recordConfiguration) :
		m_filePath(recordConfiguration.getFilePath()),
		m_cloudType(recordConfiguration.getRecordCloudType()),
		m_enabled(false),
		m_fileFormat(recordConfiguration.getRecordFileFormat()),
		m_fileName(recordConfiguration.getFileName())
	{
	}

	RecordingConfiguration(RecordCloudType cloudType, RecordingFileFormat format) : 
		m_filePath(), 
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
		bool filePathSet = !m_filePath.empty();
		return fileNameLengthExists && filePathSet;
	}

	std::string getFilePath()
	{
		return m_filePath;
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

	LPTSTR getFileName()
	{
		return m_fileName;
	}

	void setFileName(LPTSTR fileName)
	{
		m_fileName = fileName;
		recordPathOrFileNameChanged(m_cloudType);
	}

	void setFilePath(std::string filePath)
	{
		m_filePath = filePath;
		recordPathOrFileNameChanged(m_cloudType);
	}

	void setFilePath(LPTSTR filePath)
	{
		//A std:string  using the char* constructor.
		char ch[MAX_PATH];
		char DefChar = ' ';
		WideCharToMultiByte(CP_ACP, 0, filePath, -1, ch, MAX_PATH, &DefChar, NULL);

		m_filePath = std::string(ch);
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

	boost::signal<void(RecordCloudType, bool)> recordConfigurationStatusChanged;
	boost::signal<void(RecordCloudType)>	recordPathOrFileNameChanged;
private:
	std::string			m_filePath;
	RecordCloudType		m_cloudType;
	RecordingFileFormat	m_fileFormat;
	LPTSTR				m_fileName;
	bool				m_enabled;
};

