#pragma once

#include <string>
#include "stdafx.h"
#include <boost/signal.hpp>
#include <boost/bind.hpp>
#include <atlstr.h>
#include "RecordingConfiguration.h"
#include <regex>
#include <algorithm>
struct CloudFile{
	//RecordingFileFormat format;
	std::string fileName;
	std::string fullFilePath;
};
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
	}

	bool isPlaybackConfigurationValid()
	{
		if (!m_enabled){
			return true;
		}
		return m_foundCloudFiles.size() > 0;
	}

	CString getFilePathCString()
	{
		return m_filePath;
	}
	
	bool isEnabled()
	{
		return m_enabled;
	}

	RecordingFileFormat getRecordFileFormat()
	{
		return m_fileFormat;
	}

	LPTSTR getFileName()
	{
		return m_fileName.GetBuffer(0);
	}

	void setFullFilePath(CString filePath)
	{
		if (filePath == m_filePath){
			return;
		}
		m_filePath = filePath;
		findFilesAtPath();
		playbackConfigurationChanged();
	}
	
	void setEnabled(bool enabled)
	{
		m_enabled = enabled;
		playbackConfigurationChanged();
	}



	RecordCloudType getRecordCloudType(){
		return m_cloudType;
	}

	CString getFirstPlaybackFile()
	{
		if (m_foundCloudFiles.size() > 0){
			CString result(m_foundCloudFiles[0].fileName.c_str());
			return result;
		}
		CString result = "";
		return result;
	}

	std::vector<CloudFile> getCloudFilesToPlay()
	{
		return m_foundCloudFiles;
	}

	boost::signal<void(void)> playbackConfigurationChanged;
private:

	std::string getFilePath()
	{
		CT2CA pszConvertedAnsiString(m_filePath);
		std::string strStd(pszConvertedAnsiString);
		return strStd;
	}

	void findFilesAtPath()
	{
		std::string filePath = getFilePath();
		m_foundCloudFiles.clear();
		if (!boost::filesystem::is_directory(filePath)){
			return;
		}
		boost::filesystem::directory_iterator dirIterator;
		int index = 0;
		for (boost::filesystem::directory_iterator i(filePath); i != dirIterator; ++i){
			if (!boost::filesystem::is_regular_file(i->status())){
				continue;
			}
			auto fileExtension = i->path().extension().string();
			
			if (fileExtension == ".pcd" || fileExtension == ".ply"){
				CloudFile cloudFile;
				cloudFile.fullFilePath = i->path().string();
				cloudFile.fileName = i->path().filename().string();
				m_foundCloudFiles.push_back(cloudFile);

				if (index == 0){
					if (fileExtension == ".pcd"){
						m_fileFormat = PCD;
					}
					else{
						m_fileFormat = PLY;
					}
				}

				index++;
			}
			//CloudFile cloudFile;
			//if (fileExtension == ".pcd"){
			//	cloudFile.format = PCD;
			//}
			//else if (fileExtension == ".ply"){
			//	cloudFile.format = PLY;
			//}
			//else{
			//	continue;
			//}
			
			//if (fileExtension == ".pcd" || fileExtension == ".ply"){
			//	m_foundCloudFiles.push_back(i->path().string());
			//}
		}
		std::sort(m_foundCloudFiles.begin(), m_foundCloudFiles.end(), &PlaybackConfiguration::sortByIntegerEnding);
	}
	

	static int extractCloudCountFromString(std::string input)
	{
		if (input == ""){
			return -1;
		}
		std::smatch m;
		std::regex_match(input, m, std::regex("([A-z0-9]*)_([0-9]*).(ply|pcd)"));
		if (m.size() > 2){
			std::cout << "result: " << m[2] << std::endl;
			int numberAsInt = std::stoi(m[2]);
			std::cout << "result as INT: " << numberAsInt << std::endl;
			return numberAsInt;
		}
		return -1;
	}

	static bool sortByIntegerEnding(const CloudFile& cloudFile1, const CloudFile cloudFile2)
	{
		return extractCloudCountFromString(cloudFile1.fileName) < extractCloudCountFromString(cloudFile2.fileName);
	}

	//std::vector<std::string>	m_foundCloudFiles;
	std::vector<CloudFile>	m_foundCloudFiles;
	CString						m_filePath;
	RecordingFileFormat			m_fileFormat;
	RecordCloudType				m_cloudType;
	CString						m_fileName;
	bool						m_enabled;

	
};

typedef std::shared_ptr<PlaybackConfiguration> PlaybackConfigurationPtr;
typedef std::vector<PlaybackConfigurationPtr> SharedPlaybackConfiguration;