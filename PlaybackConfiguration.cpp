#include "stdafx.h"
#include "PlaybackConfiguration.h"


PlaybackConfiguration::PlaybackConfiguration() : m_filePath(), m_enabled(false), m_wasFullPlayed(false), m_centeredReading(false)
{}

PlaybackConfiguration::PlaybackConfiguration(PlaybackConfiguration& playbackConfigurationToCopy)
{
	this->m_enabled = playbackConfigurationToCopy.m_enabled;
	this->m_fileFormat = playbackConfigurationToCopy.m_fileFormat;
	this->m_filePath = playbackConfigurationToCopy.m_filePath;
	this->m_foundCloudFiles = playbackConfigurationToCopy.m_foundCloudFiles;
	this->m_centeredReading = playbackConfigurationToCopy.m_centeredReading;
}

PlaybackConfiguration& PlaybackConfiguration::operator= (const PlaybackConfiguration& rhs)
{
	this->m_enabled		= rhs.m_enabled;
	this->m_fileFormat	= rhs.m_fileFormat;
	this->m_filePath	= rhs.m_filePath;
	this->m_foundCloudFiles = rhs.m_foundCloudFiles;
	this->m_wasFullPlayed = rhs.m_wasFullPlayed;
	this->m_centeredReading = rhs.m_centeredReading;
	return *this;
}

PlaybackConfiguration::PlaybackConfiguration(RecordingConfiguration& recordConfiguration) :
m_enabled(false),
m_fileFormat(recordConfiguration.getRecordFileFormat()),
m_centeredReading(false),
m_wasFullPlayed(false)
{
	auto fullFilePath = recordConfiguration.getFullRecordingPath();
	setFullFilePath(fullFilePath);
}

void PlaybackConfiguration::setWasFullPlayed()
{
	m_wasFullPlayed = true;
}
bool PlaybackConfiguration::wasFullPlayed()
{
	return m_wasFullPlayed;
}

bool PlaybackConfiguration::isPlaybackConfigurationValid()
{
	if (!m_enabled){
		return true;
	}
	return m_foundCloudFiles.size() > 0;
}

bool PlaybackConfiguration::operator == (PlaybackConfiguration& playbackToCompareWith)
{
	auto cloudCountOfFilesInDirectoryToPlay = playbackToCompareWith.getCloudFilesToPlayCount();
	bool sameCloudCountOfFilesInDirectoryToPlay = cloudCountOfFilesInDirectoryToPlay == getCloudFilesToPlayCount();
	if (!sameCloudCountOfFilesInDirectoryToPlay){
		return false;
	}
	if (cloudCountOfFilesInDirectoryToPlay > 0){
		auto fullPathOfFirstPlaybackFile = playbackToCompareWith.getCloudFilesToPlay()[0].fullFilePath;
		auto filepath = getCloudFilesToPlay()[0].fullFilePath;
		return fullPathOfFirstPlaybackFile == filepath;
	}
	return true;
}


bool PlaybackConfiguration::isEnabled()
{
	return m_enabled;
}

bool PlaybackConfiguration::isCenteredReading()
{
	return m_centeredReading;
}

void PlaybackConfiguration::setCenteredReading(bool enable)
{
	m_centeredReading = enable;
}

RecordingFileFormat PlaybackConfiguration::getRecordFileFormat()
{
	return m_fileFormat;
}

void PlaybackConfiguration::setFullFilePath(CString filePath)
{
	if (filePath == m_filePath){
		return;
	}
	m_filePath = filePath;
	findFilesAtPath();
	playbackConfigurationChanged();
}

void PlaybackConfiguration::setEnabled(bool enabled)
{
	m_enabled = enabled;
	playbackConfigurationChanged();
}


CString PlaybackConfiguration::getFirstPlaybackFile()
{
	if (m_foundCloudFiles.size() > 0){
		CString result(m_foundCloudFiles[0].fileName.c_str());
		return result;
	}
	CString result = "";
	return result;
}

int PlaybackConfiguration::getCloudFilesToPlayCount()
{
	return m_foundCloudFiles.size();
}

std::vector<CloudFile> PlaybackConfiguration::getCloudFilesToPlay()
{
	return m_foundCloudFiles;
}

void PlaybackConfiguration::sortCloudFilesForPlayback()
{
	std::sort(m_foundCloudFiles.begin(), m_foundCloudFiles.end(), &PlaybackConfiguration::sortByIntegerEnding);
}

std::string PlaybackConfiguration::getFilePath()
{
	CT2CA pszConvertedAnsiString(m_filePath);
	std::string strStd(pszConvertedAnsiString);
	return strStd;
}

void PlaybackConfiguration::findFilesAtPath()
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
	}
}


int PlaybackConfiguration::extractCloudCountFromString(std::string input)
{
	if (input == ""){
		return -1;
	}
	std::smatch m;
	std::regex_match(input, m, std::regex("([A-z0-9]*)_([0-9]*).(ply|pcd)"));
	if (m.size() > 2){
		int numberAsInt = std::stoi(m[2]);
		return numberAsInt;
	}
	return -1;
}

bool PlaybackConfiguration::sortByIntegerEnding(const CloudFile& cloudFile1, const CloudFile& cloudFile2)
{
	return extractCloudCountFromString(cloudFile1.fileName) < extractCloudCountFromString(cloudFile2.fileName);
}