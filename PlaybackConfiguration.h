#pragma once
#include "stdafx.h"

#include <string>
#include <atlstr.h>
#include "RecordingConfiguration.h"
#include <regex>
#include <algorithm>
#include <iostream>


struct CloudFile{
	std::string fileName;
	std::string fullFilePath;
};

/**
 * \class	PlaybackConfiguration
 *
 * \brief	PlaybackConfiguration stores all information required by the InputReader to read point
 * 			cloud files from the disc.
 */
class PlaybackConfiguration{

public:

	PlaybackConfiguration();

	/**
	 * \fn	PlaybackConfiguration::PlaybackConfiguration(RecordingConfiguration& recordConfiguration);
	 *
	 * \brief	Constructor.
	 *
	 * \param [in]	recordConfiguration	The record configuration providing the basis for most of the
	 * 				fields rquired by the PlaybackConfiguration.
	 */
	PlaybackConfiguration(RecordingConfiguration& recordConfiguration);

	/**
	 * \fn	PlaybackConfiguration::PlaybackConfiguration(PlaybackConfiguration& playbackConfigurationToCopy);
	 *
	 * \brief	Copy constructor.
	 *
	 * \param [in,out]	playbackConfigurationToCopy	The playback configuration to copy.
	 */

	PlaybackConfiguration(PlaybackConfiguration& playbackConfigurationToCopy);

	/**
	 * \fn	PlaybackConfiguration& PlaybackConfiguration::operator=(const PlaybackConfiguration& rhs);
	 *
	 * \brief	Assignment operator.
	 *
	 * \param	rhs	The right hand side.
	 *
	 * \return	A shallow copy of this object.
	 */
	PlaybackConfiguration& operator=(const PlaybackConfiguration& rhs);

	/**
	 * \fn	bool PlaybackConfiguration::operator==(PlaybackConfiguration& playbackToCompareWith);
	 *
	 * \brief	Equality operator.
	 *
	 * \param [in]	playbackToCompareWith	The playback to compare with.
	 *
	 * \return	true if both PlaybackConfigurations share the same directory path and their path contains
	 * 			the same amount of files, they are considered equivalent.
	 */
	bool operator==(PlaybackConfiguration& playbackToCompareWith);

	/**
	 * \fn	bool PlaybackConfiguration::isPlaybackConfigurationValid();
	 *
	 * \brief	Query if this object is playback configuration valid.
	 *
	 * \return	true if playback configuration valid, false if not.
	 */

	bool isPlaybackConfigurationValid();

	/**
	 * \fn	bool PlaybackConfiguration::isEnabled();
	 *
	 * \brief	Query if the PlaybackConfiguration is enabled.
	 *
	 * \return	true if enabled, false if not.
	 */
	bool isEnabled();

	/**
	 * \fn	RecordingFileFormat PlaybackConfiguration::getRecordFileFormat();
	 *
	 * \brief	Gets record file format.
	 *
	 * \return	The record file format.
	 */

	RecordingFileFormat getRecordFileFormat();

	/**
	 * \fn	void PlaybackConfiguration::setFullFilePath(CString filePath);
	 *
	 * \brief	Sets full file path of the directory containing the point cloud files.
	 *
	 * \param	filePath	Full pathname of the directory.
	 */

	void setFullFilePath(CString filePath);

	/**
	 * \fn	void PlaybackConfiguration::setEnabled(bool enabled);
	 *
	 * \brief	Sets an enabled.
	 *
	 * \param	enabled	true to enable, false to disable.
	 */

	void setEnabled(bool enabled);

	/**
	 * \fn	RecordCloudType PlaybackConfiguration::getRecordCloudType();
	 *
	 * \brief	Gets record cloud type.
	 *
	 * \return	The record cloud type.
	 */

	RecordCloudType getRecordCloudType();

	/**
	 * \fn	CString PlaybackConfiguration::getFirstPlaybackFile();
	 *
	 * \brief	Gets the first playback file.
	 *
	 * \return	The first playback file.
	 */

	CString getFirstPlaybackFile();

	/**
	 * \fn	int PlaybackConfiguration::getCloudFilesToPlayCount();
	 *
	 * \brief	Gets amount of cloud files to play.
	 *
	 * \return	The amount of point cloud files to play.
	 */

	int getCloudFilesToPlayCount();

	/**
	 * \fn	std::vector<CloudFile> PlaybackConfiguration::getCloudFilesToPlay();
	 *
	 * \brief	Gets cloud files to play.
	 *
	 * \return	The cloud files to play.
	 */

	std::vector<CloudFile> getCloudFilesToPlay();

	/**
	 * \fn	void PlaybackConfiguration::sortCloudFilesForPlayback();
	 *
	 * \brief	Sort cloud files for playback according to the number contained
	 * 			in the point cloud file name e.g. HDFace_0001.ply
	 */

	void sortCloudFilesForPlayback();


	/** \brief	The playback configuration changed signal. */
	boost::signal<void(void)> playbackConfigurationChanged;

	/**
	 * \fn	void PlaybackConfiguration::setWasFullPlayed();
	 *
	 * \brief	Sets was full played and therefore contained in the buffer.
	 */

	void setWasFullPlayed();

	/**
	 * \fn	bool PlaybackConfiguration::wasFullPlayed();
	 *
	 * \brief	Determines if the PlaybackConfiguration was already fully played.
	 *
	 * \return	true if it was played, otherwise false.
	 */

	bool wasFullPlayed();

private:

	/**
	 * \fn	std::string PlaybackConfiguration::getFilePath();
	 *
	 * \brief	Gets file path as std::string
	 *
	 * \return	The file path.
	 */

	std::string getFilePath();

	/**
	 * \fn	void PlaybackConfiguration::findFilesAtPath();
	 *
	 * \brief	Searches for the first files in the specified directory.
	 */

	void findFilesAtPath();

	/**
	 * \fn	static int PlaybackConfiguration::extractCloudCountFromString(std::string input);
	 *
	 * \brief	Extracts the cloud count from the given name.
	 *
	 * \param	input	The input name.
	 *
	 * \return	The number contained in the string.
	 */

	static int extractCloudCountFromString(std::string input);

	/**
	 * \fn	static bool PlaybackConfiguration::sortByIntegerEnding(const CloudFile& cloudFile1, const CloudFile& cloudFile2);
	 *
	 * \brief	Compare method for sorting by name ending of point cloud files.
	 *
	 * \param	cloudFile1	The first cloud file.
	 * \param	cloudFile2	The second cloud file.
	 *
	 * \return	true if cloudFile1 < cloudFile2, otherwise false
	 */

	static bool sortByIntegerEnding(const CloudFile& cloudFile1, const CloudFile& cloudFile2);


	/** \brief	The found cloud files. */
	std::vector<CloudFile>		m_foundCloudFiles;


	/** \brief	path to the directory containing the point cloud files. */
	CString						m_filePath;

	/** \brief	The detected file format. */
	RecordingFileFormat			m_fileFormat;

	/** \brief	Type of the cloud. */
	RecordCloudType				m_cloudType;


	/** \brief	Filename of the file. */
	CString						m_fileName;

	/** \brief	true to enable, false to disable the PlaybackConfiguration. */
	bool						m_enabled;

	/** \brief	true if was full played. */
	bool						m_wasFullPlayed;

};

typedef std::shared_ptr<PlaybackConfiguration> PlaybackConfigurationPtr;
typedef std::vector<PlaybackConfigurationPtr> SharedPlaybackConfiguration;