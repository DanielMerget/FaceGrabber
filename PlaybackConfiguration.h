#pragma once

#include <string>
#include "stdafx.h"
#include <boost/signal.hpp>
#include <boost/bind.hpp>
#include <atlstr.h>
#include "RecordingConfiguration.h"

class PlaybackConfiguration{

public:
	PlaybackConfiguration() : m_filePath(), m_enabled(false)
	{}


	PlaybackConfiguration(RecordingConfiguration& recordConfiguration) :
		m_enabled(false),
		m_fileFormat(recordConfiguration.getRecordFileFormat()),
		m_fileName(recordConfiguration.getFileName()),
		m_cloudType(recordConfiguration.getRecordCloudType())
	{
		auto fullFilePath = recordConfiguration.getFullRecordingPath();
		setFullFilePath(fullFilePath);
		findFilesAtPath();
	}
	
	PlaybackConfiguration(PlaybackConfiguration& playbackConfiguration) :
		m_filePath(playbackConfiguration.getFilePathCString()),
		m_enabled(false),
		m_fileFormat(playbackConfiguration.getRecordFileFormat()),
		m_fileName(playbackConfiguration.getFileName()),
		m_cloudType(playbackConfiguration.getRecordCloudType())
	{
	}


	PlaybackConfiguration(PlaybackConfiguration&& playbackConfiguration) :
		m_filePath(playbackConfiguration.getFilePathCString()),
		m_enabled(false),
		m_fileFormat(playbackConfiguration.getRecordFileFormat()),
		m_fileName(playbackConfiguration.getFileName()),
		m_cloudType(playbackConfiguration.getRecordCloudType())
	{
	}


	bool isPlaybackConfigurationValid()
	{
		if (!m_enabled){
			return true;
		}
		bool fileNameLengthExists = (wcslen(m_fileName) > 0);
		bool filePathSet = !m_filePath.IsEmpty();
		return fileNameLengthExists && filePathSet;
	}

	CString getFilePathCString()
	{
		return m_filePath;
	}

	std::string getFilePath()
	{
		CT2CA pszConvertedAnsiString(m_filePath);
		std::string strStd(pszConvertedAnsiString);
		return strStd;
	}

	bool isEnabled()
	{
		return m_enabled;
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
		playbackConfigurationChanged();
	}

	void setFullFilePath(CString filePath)
	{
		m_filePath = filePath;
		findFilesAtPath();
		playbackConfigurationChanged();
	}

	void setFullFilePath(std::string filePath)
	{
		setFullFilePath(CString(filePath.c_str()));
	}
/*
	void setFilePath(std::string filePath)
	{
		auto fullFilePath = RecordingConfiguration::getFullRecordingPathForCloudType(m_cloudType, filePath);
		setFullFilePath(fullFilePath);
		findFilesAtPath();
		playbackConfigurationChanged();
	}

	void setFilePath(LPTSTR filePath)
	{
		CT2CA pszConvertedAnsiString(filePath);
		std::string filePathString(pszConvertedAnsiString);
		m_filePath = CString(filePath);
		auto fullFilePath = RecordingConfiguration::getFullRecordingPathForCloudType(m_cloudType, filePathString);
		setFullFilePath(fullFilePath);
		findFilesAtPath();
		playbackConfigurationChanged();
	}*/

	void setEnabled(bool enabled)
	{
		m_enabled = enabled;
		playbackConfigurationChanged();
	}

	void setFileFormat(RecordingFileFormat fileFormat)
	{
		m_fileFormat = fileFormat;
	}

	RecordCloudType setRecordCloudType(){
		return m_cloudType;
	}

	RecordCloudType getRecordCloudType(){
		return m_cloudType;
	}

	void findFilesAtPath()
	{
		std::string filePath = getFilePath();		
		m_foundCloudFiles.clear();
		if (!boost::filesystem::is_directory(filePath)){
			return;
		}
		boost::filesystem::directory_iterator dirIterator;
		for (boost::filesystem::directory_iterator i(filePath); i != dirIterator; ++i){
			if (!boost::filesystem::is_regular_file(i->status())){
				continue;
			}
			auto fileExtension = i->path().extension().string();
			if (fileExtension == ".pcd" || fileExtension == ".ply"){
				m_foundCloudFiles.push_back(i->path().string());
			}
		}
	}

	CString getFirstPlaybackFile()
	{
		if (m_foundCloudFiles.size() > 0){
			CString result(m_foundCloudFiles[0].c_str());
			return result;
		}
		CString result = "";
		return result;
	}

	bool isValidFileConfiguration()
	{
		return m_foundCloudFiles.size() > 0;
	}

	boost::signal<void(void)> playbackConfigurationChanged;
private:
	std::vector<std::string>	m_foundCloudFiles;
	CString						m_filePath;
	RecordingFileFormat			m_fileFormat;
	RecordCloudType				m_cloudType;
	CString						m_fileName;
	bool						m_enabled;
};

typedef std::vector<std::shared_ptr<PlaybackConfiguration>> SharedPlaybackConfiguration;