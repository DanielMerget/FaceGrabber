#pragma once
#include "stdafx.h"

#include <string>
#include "stdafx.h"
#include <boost/signal.hpp>
#include <boost/bind.hpp>
#include <boost/filesystem.hpp>
#include <atlstr.h>
#include <regex>

#include "IRecordingConfiguration.h"

/**
 * \class	RecordingConfiguration
 *
 * \brief	The Recording Configuration stores all information neccessary to the KinectCloudFileWriter to save
 * 			ply and pcd files to the directory. It allows to specify the directory for the files and automatically
 * 			searches for the files. It constructs the directory path to save the point clouds in the following way:
 *			//outputfolder/timestamp/cloudtype/filename.fileformat.
 */

class RecordingConfiguration : public IRecordingConfiguration{
#define UNLIMITED_FRAMES -1;
#define FRAMES_NOT_SET 0;

public:
	RecordingConfiguration();

	/**
	 * \fn	RecordingConfiguration::RecordingConfiguration(RecordCloudType cloudType, RecordingFileFormat format);
	 *
	 * \brief	Constructor.
	 *
	 * \param	cloudType	Type of the cloud to record..
	 * \param	format   	Describes the format to use.
	 */

	RecordingConfiguration(RecordCloudType cloudType, RecordingFileFormat format);

	/**
	 * \fn	RecordingConfiguration::RecordingConfiguration(RecordingConfiguration& recordingConfiguration);
	 *
	 * \brief	Copy constructor.
	 *
	 * \param [in]	recordingConfiguration	The recording configuration.
	 */

	RecordingConfiguration(RecordingConfiguration& recordingConfiguration);

	
	static CString getFileFormatAsString(RecordingFileFormat fileFormat);

	/**
	 * \fn	bool RecordingConfiguration::isRecordConfigurationValid();
	 *
	 * \brief	Query if record configuration is valid: Valid only if we have a 
	 * 			valid path, name and number of frames to captures are set.
	 *
	 * \return	true if record configuration valid, false if not.
	 */
	bool isRecordConfigurationValid();

	/**
	 * \fn	CString RecordingConfiguration::getFilePathCString();
	 *
	 * \brief	Gets file path as c-string. (for WinApi)
	 *
	 * \return	The file path c string.
	 */
	CString getFilePathCString();

	/**
	 * \fn	std::string RecordingConfiguration::getFilePath();
	 *
	 * \brief	Gets file path as std::string (for pcl)
	 *
	 * \return	The file path.
	 */
	std::string getFilePath();

	/**
	 * \fn	bool RecordingConfiguration::isEnabled();
	 *
	 * \brief	Query if Recording is enabled.
	 *
	 * \return	true if enabled, false if not.
	 */
	bool isEnabled();

	/**
	 * \fn	RecordCloudType RecordingConfiguration::getRecordCloudType();
	 *
	 * \brief	Gets cloud type to record
	 *
	 * \return	The record cloud type.
	 */
	RecordCloudType getRecordCloudType();

	/**
	 * \fn	RecordingFileFormat RecordingConfiguration::getRecordFileFormat();
	 *
	 * \brief	Gets fileformat to record
	 *
	 * \return	The record file format.
	 */
	RecordingFileFormat getRecordFileFormat();

	/**
	 * \fn	CString RecordingConfiguration::getFileNameCString();
	 *
	 * \brief	Gets file name  as c-string. (for WinApi)
	 * 			e.g. HD_Face_Cloud_*.ply
	 *
	 * \return	The file name c string.
	 */
	CString getFileNameCString();

	/**
	 * \fn	bool RecordingConfiguration::isRecordingDurationUnLimited();
	 *
	 * \brief	Query if recording duration is unlimited.
	 *
	 * \return	true if recording duration un limited, false if number of frames to record
	 * 			was set.
	 */
	bool isRecordingDurationUnLimited();

	/**
	 * \fn	void RecordingConfiguration::setMaxNumberOfFrames(int newMaxNumberOfFrames);
	 *
	 * \brief	Sets maximum number of frames to record.
	 *
	 * \param	newMaxNumberOfFrames	The new maximum number of frames.
	 */
	void setMaxNumberOfFrames(int newMaxNumberOfFrames);

	/**
	 * \fn	int RecordingConfiguration::getMaxNumberOfFrames();
	 *
	 * \brief	Gets the maximum number of frames to record. 
	 * 			IF: UNLIMITED_FRAMES: recording is not limited
	 *
	 * \return	The maximum number of frames.
	 */
	int getMaxNumberOfFrames();

	/**
	 * \fn	std::string RecordingConfiguration::getFileNameString();
	 *
	 * \brief	Gets file name string.
	 *
	 * \return	The file name string.
	 */
	std::string getFileNameString();

	
	void setFileName(LPTSTR fileName);
	void setFilePath(std::string filePath);

	void setFilePath(LPTSTR filePath);

	void setEnabled(bool enabled);

	void setFileFormat(RecordingFileFormat fileFormat);

	void setTimeStampFolderName(CString folderName);

	CString getTimeStampFolderName();

	CString getFullRecordingPath();

	std::string getFullRecordingPathString();
	
	static CString getSubFolderNameForCloudType(RecordCloudType cloudType);

	static CString getFullRecordingPathForCloudType(RecordCloudType cloudType, CString outputFolder, CString timeStampFolderName);

	int getThreadCountToStart();

	void setThreadCountToStart(int threadsCount);

	boost::signal<void(RecordCloudType, bool)> recordConfigurationStatusChanged;
	boost::signal<void(RecordCloudType)>	recordPathOrFileNameChanged;
private:
	CString getDefauldFileName();

	void setDefaultFileName();

	std::vector<std::string> m_foundCloudFiles;

	//outputfolder/timestamp/cloudtype/filename.fileformat

	CString					m_outputFolder;
	CString					m_timeStampFolderName;
	CString					m_fileName;
	RecordCloudType			m_cloudType;
	RecordingFileFormat		m_fileFormat;
	int						m_maxNumberOfFrames;
	bool					m_enabled;
	int						m_threadsCount;
};
typedef std::shared_ptr<RecordingConfiguration> RecordingConfigurationPtr;
typedef std::vector<RecordingConfigurationPtr> SharedRecordingConfiguration;
